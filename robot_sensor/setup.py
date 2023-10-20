from setuptools import setup

package_name = 'robot_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thayjes',
    maintainer_email='thayjes.srivas@gmail.com',
    description='Robot arm sensor client server',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_service = robot_sensor.service_arm_sensor:main',
            'sensor_client = robot_sensor.client_sensor:main',
        ],
    },
)
