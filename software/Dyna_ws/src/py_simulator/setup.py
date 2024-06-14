from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'py_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'),glob('rviz/*')),
        (os.path.join('share', package_name, 'stl'),glob('stl/*')),
        (os.path.join('share', package_name, 'urdf'),glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'),glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='udesa',
    maintainer_email='tadeo.casiraghi@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim = py_simulator.py_bullet_executer:main',
        ],
    },
)
