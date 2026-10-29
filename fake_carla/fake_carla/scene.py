"""A tiny fake world, in plain numpy (no ROS), that produces CARLA-shaped sensor data.

The world: a straight road along +x (3.5 m each side of y = 0), lane markings, grass, two long
walls at y = +-12 m, and a red parked car in the lane ahead. When the ego car passes the
parked car, it moves 40 m further ahead, so there's always something to see.

Camera and lidar are both ray-cast against the same objects, so they agree with each other.
Everything uses ROS axes (x forward, y left, z up), meters, radians.

Matches the real rig in carla_sensors/config/objects.json:
  camera  800 x 600, 90 deg FOV, mounted at (2.0, 0, 1.6), published as bgra8
  lidar   32 channels, -26.8..+2 deg, 50 m range, mounted at (0, 0, 2.4),
          points relative to the lidar, intensity = exp(-0.004 * distance) like CARLA
"""
import math
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

# ---------------------------------------------------------------- rig (keep in sync with objects.json)
CAMERA_W, CAMERA_H, CAMERA_FOV = 800, 600, 90.0
CAMERA_POS = (2.0, 0.0, 1.6)
LIDAR_POS = (0.0, 0.0, 2.4)
LIDAR_CHANNELS, LIDAR_UPPER, LIDAR_LOWER, LIDAR_RANGE = 32, 2.0, -26.8, 50.0
LIDAR_AZIMUTH_STEPS = 500        # 32 x 500 = 16,000 rays per scan, like the real 320k pts/s at 20 Hz
WHEELBASE = 2.9                  # [m] Tesla Model 3
MAX_STEER = math.radians(70.0)   # wheel angle at steer = 1.0

GROUND, OBSTACLE, WALL = 1, 2, 3   # what a ray hit (0 = nothing)

Box = Tuple[float, float, float, float, float, float]  # xmin, xmax, ymin, ymax, zmin, zmax (world)


def camera_K() -> np.ndarray:
    """3x3 intrinsics of a perfect pinhole, as CARLA's camera_info reports them."""
    f = CAMERA_W / (2.0 * math.tan(math.radians(CAMERA_FOV) / 2.0))
    return np.array([[f, 0.0, CAMERA_W / 2.0], [0.0, f, CAMERA_H / 2.0], [0.0, 0.0, 1.0]])


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0     # [rad], counter-clockwise positive
    speed: float = 0.0   # [m/s]
    yaw_rate: float = 0.0


@dataclass
class FakeScene:
    car: VehicleState = field(default_factory=VehicleState)
    obstacle_ahead: float = 40.0   # [m] where the parked car sits ahead of the start
    time: float = 0.0              # [s] sim time
    render_scale: int = 2          # ray-cast every Nth camera pixel and upscale (speed); 1 = full res

    def __post_init__(self):
        self.obstacle_x = self.car.x + self.obstacle_ahead
        self._camera_dirs = self._make_camera_dirs()
        self._lidar_dirs = self._make_lidar_dirs()

    # ------------------------------------------------ world
    WALL_Y, WALL_HEIGHT = 12.0, 4.0   # walls are planes at y = +-12 m, 4 m tall
    CAR_SIZE = (4.7, 2.0, 1.45)        # parked car length, width, height [m]

    def obstacle_box(self) -> Box:
        length, width, height = self.CAR_SIZE
        return (self.obstacle_x, self.obstacle_x + length, -width / 2, width / 2, 0.0, height)

    # ------------------------------------------------ physics: CARLA-style controls in
    def step(self, dt: float, throttle: float, steer_carla: float, brake: float, reverse: bool = False):
        """Advance by dt. steer_carla uses CARLA's sign (+ = right), like vehicle_control_cmd."""
        c = self.car
        throttle = min(max(throttle, 0.0), 1.0)
        brake = min(max(brake, 0.0), 1.0)
        accel = 4.0 * throttle - 8.0 * brake * (1.0 if c.speed > 0 else 0.0) - 0.05 * c.speed
        if reverse:
            accel = -4.0 * throttle + 8.0 * brake * (1.0 if c.speed < 0 else 0.0) - 0.05 * c.speed
        new_speed = c.speed + accel * dt
        if not reverse and c.speed >= 0 and new_speed < 0:
            new_speed = 0.0  # brakes stop the car, they don't reverse it
        c.speed = new_speed
        wheel = -min(max(steer_carla, -1.0), 1.0) * MAX_STEER   # to ROS sign (+ = left)
        c.yaw_rate = c.speed / WHEELBASE * math.tan(wheel)
        c.yaw = (c.yaw + c.yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        c.x += c.speed * math.cos(c.yaw) * dt
        c.y += c.speed * math.sin(c.yaw) * dt
        self._advance(dt)

    def cruise(self, dt: float, speed: float):
        """Drive straight at a fixed speed (used when nobody is sending controls)."""
        c = self.car
        c.speed, c.yaw_rate = speed, 0.0
        c.x += speed * math.cos(c.yaw) * dt
        c.y += speed * math.sin(c.yaw) * dt
        self._advance(dt)

    def _advance(self, dt: float):
        self.time += dt
        if self.car.x > self.obstacle_x + self.CAR_SIZE[0]:
            self.obstacle_x = self.car.x + self.obstacle_ahead

    # ------------------------------------------------ ray casting
    def _to_world(self, dirs_car: np.ndarray, mount) -> Tuple[np.ndarray, np.ndarray]:
        c, s = math.cos(self.car.yaw), math.sin(self.car.yaw)
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        origin = np.array([self.car.x, self.car.y, 0.0]) + rot @ np.array(mount)
        return origin, dirs_car @ rot.T.astype(np.float32)

    def raycast(self, origin: np.ndarray, dirs: np.ndarray, max_t: float = np.inf):
        """Nearest hit for each ray. Returns (t, kind) with t = distance along the unit ray."""
        ox, oy, oz = (float(v) for v in origin)
        dx, dy, dz = dirs[:, 0], dirs[:, 1], dirs[:, 2]
        inf = np.float32(np.inf)
        with np.errstate(divide="ignore", invalid="ignore"):
            # ground plane z = 0
            best_t = np.where(dz < 0, -oz / dz, inf).astype(np.float32)
            kind = np.where(np.isfinite(best_t), GROUND, 0).astype(np.int8)
            # walls: planes y = +-WALL_Y, between z = 0 and WALL_HEIGHT
            wall_y = np.where(dy > 0, self.WALL_Y, -self.WALL_Y)
            t_wall = (wall_y - oy) / dy
            z_wall = oz + t_wall * dz
            t_wall = np.where((t_wall > 0) & (z_wall >= 0) & (z_wall <= self.WALL_HEIGHT), t_wall, inf)
            closer = t_wall < best_t
            best_t[closer], kind[closer] = t_wall[closer], WALL
            # parked car: slab test, only for rays pointing roughly at it
            x0, x1, y0, y1, z0, z1 = self.obstacle_box()
            cx, cy = (x0 + x1) / 2 - ox, (y0 + y1) / 2 - oy
            dist = math.hypot(cx, cy)
            radius = math.hypot(x1 - x0, y1 - y0, z1 - z0) / 2
            if dist > radius:
                cos_max = math.cos(math.asin(min(radius / dist, 1.0)))
                near = (dx * cx + dy * cy) / dist >= cos_max * 0.98
            else:
                near = np.ones(len(dirs), dtype=bool)
            idx = np.nonzero(near)[0]
            if len(idx):
                d = dirs[idx]
                lo = (np.array([x0, y0, z0]) - origin) / d
                hi = (np.array([x1, y1, z1]) - origin) / d
                t_near = np.nanmax(np.minimum(lo, hi), axis=1)
                t_far = np.nanmin(np.maximum(lo, hi), axis=1)
                t_box = np.where((t_near <= t_far) & (t_near > 0), t_near, np.inf)
                closer = t_box < best_t[idx]
                best_t[idx[closer]], kind[idx[closer]] = t_box[closer], OBSTACLE
        miss = best_t > max_t
        best_t[miss], kind[miss] = inf, 0
        return best_t, kind

    # ------------------------------------------------ camera: bgra8 bytes, like the bridge
    def _make_camera_dirs(self) -> np.ndarray:
        K = camera_K()
        n = self.render_scale
        u, v = np.meshgrid(np.arange(0, CAMERA_W, n) + n / 2, np.arange(0, CAMERA_H, n) + n / 2)
        d = np.stack([np.full(u.shape, K[0, 0]), -(u - K[0, 2]), -(v - K[1, 2])], axis=-1).reshape(-1, 3)
        return (d / np.linalg.norm(d, axis=1, keepdims=True)).astype(np.float32)

    def render_camera(self) -> np.ndarray:
        """H x W x 4 uint8 array in B, G, R, A order (call .tobytes() for Image.data)."""
        origin, dirs = self._to_world(self._camera_dirs, CAMERA_POS)
        t, kind = self.raycast(origin, dirs)
        tt = np.where(np.isfinite(t), t, 0.0).astype(np.float32)
        wx = origin[0] + dirs[:, 0] * tt
        wy = np.abs(origin[1] + dirs[:, 1] * tt)
        g = kind == GROUND
        color = np.zeros(len(dirs), dtype=np.uint8)                 # palette index; 0 = sky
        color[g] = 1                                                # grass
        color[g & (wy < 3.5)] = 2                                   # asphalt
        color[g & (((wy < 0.12) & (np.mod(wx, 6.0) < 3.0)) | (np.abs(wy - 3.35) < 0.1))] = 3  # lines
        color[kind == OBSTACLE] = 4
        color[kind == WALL] = 5
        bgr = _PALETTE_BGR[color]
        shade = np.where(kind > 0, np.clip(1.0 - tt / 150.0, 0.45, 1.0), 1.0).astype(np.float32)
        n = self.render_scale
        small = np.empty((len(dirs), 4), dtype=np.uint8)
        small[:, :3] = (bgr * shade[:, None]).astype(np.uint8)
        small[:, 3] = 255
        small = small.reshape(CAMERA_H // n, CAMERA_W // n, 4)
        if n == 1:
            return small
        return np.ascontiguousarray(np.repeat(np.repeat(small, n, axis=0), n, axis=1))

    # ------------------------------------------------ lidar: N x 4 float32, like the bridge
    def _make_lidar_dirs(self) -> np.ndarray:
        elev = np.radians(np.linspace(LIDAR_UPPER, LIDAR_LOWER, LIDAR_CHANNELS))
        azim = np.linspace(-math.pi, math.pi, LIDAR_AZIMUTH_STEPS, endpoint=False)
        e, a = np.meshgrid(elev, azim, indexing="ij")
        d = np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], axis=-1).reshape(-1, 3)
        return d.astype(np.float32)

    def lidar_points(self, rng: np.random.Generator = None, noise_stddev: float = 0.0) -> np.ndarray:
        """(x, y, z, intensity) per hit, relative to the lidar. Rays that hit nothing in range are dropped."""
        origin, dirs_world = self._to_world(self._lidar_dirs, LIDAR_POS)
        t, kind = self.raycast(origin, dirs_world, max_t=LIDAR_RANGE)
        keep = kind > 0
        if noise_stddev > 0.0:
            rng = rng or np.random.default_rng()
            t = t + rng.normal(0.0, noise_stddev, size=t.shape)
        pts = self._lidar_dirs[keep] * t[keep, None]   # sensor frame = car frame axes
        intensity = np.exp(-0.004 * t[keep])
        return np.hstack([pts, intensity[:, None]]).astype(np.float32)


# sky, grass, asphalt, lane lines, parked car (red), walls -- in B, G, R order like CARLA
_PALETTE_BGR = np.array([
    (235, 190, 135), (60, 120, 70), (88, 85, 85), (230, 230, 230), (30, 30, 180), (140, 160, 170),
], dtype=np.float32)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) for a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
