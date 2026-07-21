from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "pinkk_mycobot_bridge"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="juwon",
    maintainer_email="juwon@example.com",
    description="MyCobot280 state, ChArUco TF and simple MoveIt execution bridge.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "charuco_tf_publisher = pinkk_mycobot_bridge.charuco_tf_publisher:main",
            "joint_state_publisher = pinkk_mycobot_bridge.joint_state_publisher:main",
            "trajectory_bridge = pinkk_mycobot_bridge.trajectory_bridge:main",
        ],
    },
)
