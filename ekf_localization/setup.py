from setuptools import setup
from glob import glob
import os

package_name = 'ekf_localization'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project Team',
    maintainer_email='team@example.com',
    description='Wheel odometry and IMU fusion plus EKF evaluation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_covariance_adapter = ekf_localization.imu_covariance_adapter:main',
            'record_ekf = ekf_localization.record_ekf:main',
            'analyze_ekf = ekf_localization.analyze_ekf:main',
            'run_ekf_experiment = ekf_localization.run_ekf_experiment:main',
        ],
    },
)
