from setuptools import find_packages, setup

package_name = 'pca9685_multiples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/servo_serial_test.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/servo_test.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ikbal',
    maintainer_email='ikbal@todo.todo',
    description='Drive PCA9685 servos on STM32 directly over serial (no micro-ROS), with S-curve smoothing',
    license='TODO',
    entry_points={
        'console_scripts': [
            'servo_serial_bridge = pca9685_multiples.servo_serial_bridge:main',
        ],
    },
)