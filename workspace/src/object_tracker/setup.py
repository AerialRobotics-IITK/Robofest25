from setuptools import find_packages, setup
import os

package_name = 'object_tracker'

# Ensure trajectory.csv is included in package data
def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Include trajectory.csv from current directory
    if os.path.exists('trajectory.csv'):
        data_files.append(('share/' + package_name, ['trajectory.csv']))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools', 'picamera2'],
    zip_safe=True,
    maintainer='azidozide',
    maintainer_email='pradyumn.vik@gmail.com',
    description='Object tracking with drone control and trajectory following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_sitl = object_tracker.tracker_sitl:main',
            'drone_control = object_tracker.controls_drone_tracking:main',
            'tracker_rpi = object_tracker.tracker_rpi_numpy:main',
            'tracker_soumya = object_tracker.tracker_soumya:main',
            'tracker_rishabh = object_tracker.tracker_rishabh:main',
            'run_demo = object_tracker.task_4_drone_node:main',
        ],
    },
)
