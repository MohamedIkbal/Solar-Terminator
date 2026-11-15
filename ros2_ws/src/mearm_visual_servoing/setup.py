from setuptools import find_packages, setup

package_name = 'mearm_visual_servoing'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mearm_visual_servoing.launch.py']),
        ('share/' + package_name + '/config', ['config/mearm_visual_servoing_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='mearm_visual_servoing ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mearm_visual_servoing_node = mearm_visual_servoing.mearm_visual_servoing_node:main',
        ],
    },
)
