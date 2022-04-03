from setuptools import setup
from glob import glob

package_name = "point_cloud_registration"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name + "/sdf/worlds", glob("sdf/worlds/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Marina-Banov",
    maintainer_email="m.banov7@gmail.com",
    description="This package uses a Turtlebot3 robot with an Intel Realsense D435i depth camera and tries to create "
                "point clouds of Gazebo environments.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"get_pcd = {package_name}.get_pcd:main",
            f"testing = {package_name}.testing:main",
        ],
    },
)

