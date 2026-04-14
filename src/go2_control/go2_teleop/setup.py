from setuptools import find_packages, setup

package_name = 'go2_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hunter',
    maintainer_email='hunter@todo.todo',
    description='Teleoperation interfaces for the Unitree Go2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joy_teleop_node = go2_teleop.joy_teleop_node:main',
            'keyboard_teleop_node = go2_teleop.keyboard_teleop_node:main',
        ],
    },
)
