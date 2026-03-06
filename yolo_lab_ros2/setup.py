from setuptools import setup

package_name = "yolo_lab_ros2"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/yolo_annotator.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RSS",
    maintainer_email="",
    description="YOLO lab",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_annotator = yolo_lab_ros2.yolo_annotator_node:main",
        ],
    },
)
