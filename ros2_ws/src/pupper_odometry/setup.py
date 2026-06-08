from setuptools import find_packages, setup

package_name = 'pupper_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/baselink_to_odom.yaml', 'config/imu_madgwick.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathankau',
    maintainer_email='nathankau@gmail.com',
    description='Dead-reckoning odometry from cmd_vel for Pupper v3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dead_reckoning_node = pupper_odometry.dead_reckoning_node:main',
            'imu_madgwick_node = pupper_odometry.imu_madgwick_node:main',
        ],
    },
)
