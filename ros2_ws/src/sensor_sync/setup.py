from setuptools import setup, find_packages

package_name = 'sensor_sync'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  #중요
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunzi',
    maintainer_email='yunzi@todo.todo',
    description='Sensor synchronization node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sync_imu_lidar = sensor_sync.sync_imu_lidar:main',
            'ekf_localization = sensor_sync.ekf_localization:main',
            
        ],
    },
)
