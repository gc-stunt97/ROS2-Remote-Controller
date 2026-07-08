import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'aira_dashboard'

setup(
    name=package_name,
    version='0.0.1',
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
    maintainer_email='e.mancinelli@brunelleschi.ai',
    description='Dashboard di guida della base mobile AIRA sul controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "aira_dashboard = aira_dashboard.dashboard:main",
        ],
    },
)
