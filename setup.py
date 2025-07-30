import os
from setuptools import find_packages, setup

package_name = 'sim_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=['sim_drone'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), ['msg/DroneStatus.msg']),
        (os.path.join('share', package_name, 'launch'), ['launch/sim.launch.py']),
    ],	
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artur',
    maintainer_email='arturgsimao@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_node = sim_drone.drone_node:main',
            'lidar_node = sim_drone.lidar_node:main',
            'obstaculo = sim_drone.obstaculo:main'
        ],
    },
)
