import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'aira_bench'

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
    description='Banco dei micro AIRA: il telecomando finge di essere il mini PC.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "aira_bench = aira_bench.bench:main",
        ],
    },
)
