import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'nav_eval'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Person 5',
    maintainer_email='you@example.com',
    description='Navigation evaluation harness',
    license='MIT',
    entry_points={
        'console_scripts': [
            'run_eval = nav_eval.run_eval:main',
            'analyze = nav_eval.analyze:main',
            'fake_data = nav_eval.fake_data:main',
        ],
    },
)
