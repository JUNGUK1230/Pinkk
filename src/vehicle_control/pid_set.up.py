from setuptools import find_packages, setup


package_name = "pinky_bringup"


setup(
    name=package_name,
    version="0.0.0",

    packages=find_packages(
        exclude=["test"],
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="user",
    maintainer_email="user@example.com",

    description="Smooth PID path follower for Pinky robot",

    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            (
                "pid_path_follower = "
                "pinky_bringup.pid_path_follower_smooth:main"
            ),
        ],
    },
)