"""ROS 2 node: takes the raw sensor topics from the CARLA ROS bridge and republishes them
under /sensors/*, so perception code never has to know it's talking to CARLA.

The bridge itself does the "pull data out of CARLA" part: it reads config/objects.json and
publishes every sensor there as /carla/ego_vehicle/<id>/... once per sim tick (20 Hz).

Inputs (subscriptions, from the CARLA ROS bridge)
  /carla/ego_vehicle/rgb_front/image        sensor_msgs/Image        front camera, 800x600, bgra8
  /carla/ego_vehicle/rgb_front/camera_info  sensor_msgs/CameraInfo   front camera intrinsics
  /carla/ego_vehicle/lidar                  sensor_msgs/PointCloud2  32-channel lidar, 50 m

Outputs (publishers, for perception and anyone else)
  /sensors/front_camera/image               sensor_msgs/Image
  /sensors/front_camera/camera_info         sensor_msgs/CameraInfo
  /sensors/lidar                            sensor_msgs/PointCloud2

Right now every message passes straight through. Put processing in the process_* methods.

TODO:
- Decide what processing belongs here vs. in perception (resize, crop, rgb conversion,
  ground removal, range/ROI filtering, downsampling)
- Warn when a sensor stops publishing
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

BRIDGE = "/carla/ego_vehicle"  # role_name of the ego vehicle in the bridge launch


class SensorsNode(Node):
    def __init__(self):
        super().__init__("sensors")

        # Raw topics from the bridge
        self.create_subscription(Image, f"{BRIDGE}/rgb_front/image", self.on_image, 10)
        self.create_subscription(CameraInfo, f"{BRIDGE}/rgb_front/camera_info", self.on_camera_info, 10)
        self.create_subscription(PointCloud2, f"{BRIDGE}/lidar", self.on_lidar, 10)

        # Our topics
        self.image_pub = self.create_publisher(Image, "/sensors/front_camera/image", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/sensors/front_camera/camera_info", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, "/sensors/lidar", 10)

    # ------------------------------------------------ callbacks: process, then republish
    def on_image(self, msg: Image):
        self.image_pub.publish(self.process_image(msg))

    def on_camera_info(self, msg: CameraInfo):
        self.camera_info_pub.publish(msg)  # TODO: update if process_image resizes/crops

    def on_lidar(self, msg: PointCloud2):
        self.lidar_pub.publish(self.process_lidar(msg))

    # ------------------------------------------------ processing hooks (pass-through for now)
    def process_image(self, msg: Image) -> Image:
        """TODO. To work on pixels (needs ros-humble-cv-bridge; add it to package.xml):
            from cv_bridge import CvBridge
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")      # numpy array, HxWx3
            ...
            out = bridge.cv2_to_imgmsg(frame, "bgr8")
            out.header = msg.header
            return out
        """
        return msg

    def process_lidar(self, msg: PointCloud2) -> PointCloud2:
        """TODO. To work on points (sensor_msgs_py ships with ROS 2 Humble):
            from sensor_msgs_py import point_cloud2
            points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z", "intensity"))
            ...
            return point_cloud2.create_cloud(msg.header, msg.fields, points)
        """
        return msg


def main():
    rclpy.init()
    node = SensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
