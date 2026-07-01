import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'joypad_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giulio',
    maintainer_email='giulio97.cassano@gmail.com',
    description='Controller remoto joystick per l\'esapode RobotHex: legge lo '
                'STM32 via seriale e pubblica i dati dei due joystick su ROS2.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joypad_node = joypad_controller.joy_node:main",
            "joypad_gui_app = joypad_controller.joypad_gui:main"
        ],
    },
)
