from setuptools import setup

package_name = 'camsense_x1'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camsense_x1.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='dungtsbb',
    maintainer_email='dungtsbb@gmail.com',
    description='ROS2 driver cho lidar Camsense X1: đọc serial, publish sensor_msgs/LaserScan',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camsense_x1_node = camsense_x1.camsense_x1_node:main',
        ],
    },
)
