from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "pinkk_handeye_automation"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["numpy", "PyYAML", "setuptools"],
    zip_safe=True,
    maintainer="juwon",
    maintainer_email="juwon@example.com",
    description="Eye-in-hand 자동 자세 이동과 Easy Handeye2 샘플 수집",
    license="MIT",
    entry_points={
        "console_scripts": [
            "auto_collect = pinkk_handeye_automation.auto_collect:main",
            "compare_calibrations = pinkk_handeye_automation.compare_calibrations:main",
            "usb_pre_approach = pinkk_handeye_automation.usb_pre_approach:main",
        ],
    },
)
