from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shivang',
    maintainer_email='shivangso23@iitk.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'follow = swarm.follow:main',
            'finder = swarm.p_finder:main',
            'tof = swarm.rangefinder:main',
            'local_pose = swarm.local_pose:main',
            'planner = swarm.planner:main'
        ],
    },
)
