from setuptools import find_packages, setup

package_name = 'ackermann_drive_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tf-transformations', 'smbus2'],
    zip_safe=True,
    maintainer='Dipanjan Maji',
    maintainer_email='mdipanjan2002@gmail.com',
    description='Ackermann steering controller for HiWonder motor driver with odometry',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'drive_node = ackermann_drive.drive_odom:main',
        ],
    },
)


