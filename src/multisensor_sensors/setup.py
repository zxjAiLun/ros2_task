from setuptools import find_packages, setup

package_name = 'multisensor_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bailan',
    maintainer_email='bailan@todo.todo',
    description='Multisensor simulated vision/audio/lidar publishers',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = multisensor_sensors.vision_node:main',
            'audio_node = multisensor_sensors.audio_node:main',
            'lidar_node = multisensor_sensors.lidar_node:main',
        ],
    },
)
