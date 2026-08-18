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
        (
            f'share/{PACKAGE_NAME}/config/robots/robot_a',
            glob('config/robots/robot_a/*.yaml'),
        ),
        (
            f'share/{PACKAGE_NAME}/config/robots/robot_b',
            glob('config/robots/robot_b/*.yaml'),
        ),
        (
            f'share/{PACKAGE_NAME}/config/robots',
            glob('config/robots/*.md'),
        ),
        (f'share/{PACKAGE_NAME}/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juwon',
    maintainer_email='juwon@example.com',
    description='USB-A 포트 인식, 접근, 영상 정렬 및 삽입을 위한 ROS 2 제어 패키지',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_publisher_node = '
            'pinkk_usb_insertion.camera_publisher_node:main',
            'yolo_keypoint_node = pinkk_usb_insertion.yolo_keypoint_node:main',
            'port_pose_node = pinkk_usb_insertion.port_pose_node:main',
            'pbvs_alignment_node = pinkk_usb_insertion.pbvs_alignment_node:main',
            'pbvs_step_executor_node = pinkk_usb_insertion.pbvs_step_executor_node:main',
            'frozen_target_executor_node = '
            'pinkk_usb_insertion.frozen_target_executor_node:main',
            'return_to_observe = '
            'pinkk_usb_insertion.return_to_observe_node:main',
        ],
    },
)
