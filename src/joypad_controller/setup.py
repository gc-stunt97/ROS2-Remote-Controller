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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giulio',
    maintainer_email='giulio@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joypad_node = joypad_controller.joy_node:main",
            "L_joypad_reader = joypad_controller.L_joy_reader:main",
            "R_joypad_reader = joypad_controller.R_joy_reader:main",
            "joypad_gui_app = joypad_controller.joypad_gui:main"
        ],
    },
)
