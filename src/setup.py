from pathlib import Path

from setuptools import find_packages, setup


PACKAGE_NAME = 'pinkk'
PACKAGE_ROOT = Path(__file__).parent


def config_files():
    """Install module configuration files while retaining their directory layout."""
    files = []
    for config_root in PACKAGE_ROOT.glob('*/config'):
        module_name = config_root.parent.name
        for path in config_root.rglob('*'):
            if path.is_file():
                relative_parent = path.parent.relative_to(config_root)
                files.append((
                    str(
                        Path('share')
                        / PACKAGE_NAME
                        / 'config'
                        / module_name
                        / relative_parent
                    ),
                    [str(path.relative_to(PACKAGE_ROOT))],
                ))
    return files


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['tests', 'tests.*']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [str(PACKAGE_ROOT / 'resource' / PACKAGE_NAME)],
        ),
        ('share/' + PACKAGE_NAME, [str(PACKAGE_ROOT / 'package.xml')]),
    ] + config_files(),
    install_requires=[
        'setuptools',
        'Flask>=3.0,<4',
    ],
    zip_safe=True,
    maintainer='PINKK',
    maintainer_email='maintainer@example.com',
    description='Smart parking system ROS 2 package.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mpc_path_follower = vehicle_control.mpc_path_follower:main',
            'mpc_visualizer = vehicle_control.mpc_visualizer:main',
            'fused_pose_estimator = vehicle_control.fused_pose_estimator:main',
            'pinky_status_led = vehicle_control.pinky_status_led:main',
            'pinky_status_lcd = vehicle_control.pinky_status_lcd:main',
            'trajectory_publisher = central_control.overhead_vision.path_planning.path_publisher:main',
            'vehicle_pose_publisher = central_control.overhead_vision.path_planning.vehicle_pose_publisher:main',
        ],
    },
)
