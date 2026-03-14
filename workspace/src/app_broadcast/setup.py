from setuptools import find_packages, setup

package_name = 'app_broadcast'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'websockets',
    ],
    zip_safe=True,
    maintainer='azidozide',
    maintainer_email='pradyumn.vik@gmail.com',
    description='ROS 2 websocket broadcaster for fleet state updates',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'broadcast_fleet_state = app_broadcast.broadcast_fleet_state:main',
        ],
    },
)
