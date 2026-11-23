import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pentographe_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreea Bujor',
    maintainer_email='andreeabb07@users.noreply.github.com',
    description='Modèles géométriques direct et inverse du pantographe (système 5 barres).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kinematics_node = pentographe_kinematics.kinematics_node:main',
        ],
    },
)
