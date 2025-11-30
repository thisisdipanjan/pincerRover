from setuptools import find_packages, setup

package_name = 'inertial_measurement'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='Dipanjan Maji',
    maintainer_email='mdipanjan2002@gmail.com',
    description='MPU6886 sensor data acquitision',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'imu_node = inertial_measurement.imu_node:main',
        ],
    },
)
