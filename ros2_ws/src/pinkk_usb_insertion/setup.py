from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = 'pinkk_usb_insertion'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{PACKAGE_NAME}']),
        (f'share/{PACKAGE_NAME}', ['package.xml', 'README.md']),
        (f'share/{PACKAGE_NAME}/launch', glob('launch/*.launch.py')),
        (f'share/{PACKAGE_NAME}/config', glob('config/*.yaml') + glob('config/*.md')),
        (f'share/{PACKAGE_NAME}/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='juwon',
    maintainer_email='juwon@example.com',
    description='USB-A 포트 인식, 접근, 영상 정렬 및 삽입을 위한 ROS 2 제어 패키지',
    license='MIT',
    entry_points={
        'console_scripts': [
            'port_pose_node = pinkk_usb_insertion.port_pose_node:main',
            'manual_detection_node = pinkk_usb_insertion.manual_detection_node:main',
            'arm_motion_node = pinkk_usb_insertion.arm_motion_node:main',
            'usb_insertion_node = pinkk_usb_insertion.usb_insertion_node:main',
        ],
    },
)
