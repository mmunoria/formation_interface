import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'formation_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='progress',
    maintainer_email='mmunoria@mtu.edu',
    description='Multi-drone formation control: terminal interface + PX4 formation node.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'interface_node = formation_interface.interface_node:main',
            'formation_node = formation_interface.formation_node:main',
        ],
    },
)
