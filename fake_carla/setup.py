from glob import glob

from setuptools import setup

package_name = "fake_carla"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "fake_bridge = fake_carla.fake_bridge:main",
        "check_sensors = fake_carla.check_sensors:main",
    ]},
)
