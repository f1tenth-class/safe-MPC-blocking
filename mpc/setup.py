from setuptools import setup
import os
from glob import glob

package_name = 'mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzangupenn, Hongrui Zheng',
    maintainer_email='zzang@seas.upenn.edu, billyzheng.bz@gmail.com',
    description='f1tenth mpc',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_node = mpc.mpc_node:main',
            'overtaking_node = mpc.overtaking_node:main',
        ],
    },
)
