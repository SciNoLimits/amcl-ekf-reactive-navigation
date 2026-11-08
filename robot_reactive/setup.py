from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_reactive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ansh',
    maintainer_email='viraniansh800@gmail.com',
    description='LiDAR obstacle detection and a reactive safety layer that complements Nav2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = robot_reactive.obstacle_detector:main',
            'reactive_controller = robot_reactive.reactive_controller:main',
            'obstacle_tool = robot_reactive.obstacle_tool:main',
            'scenario_runner = robot_reactive.scenario_runner:main',
            'plot_results = robot_reactive.plot_results:main',
        ],
    },
)
