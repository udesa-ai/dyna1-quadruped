from setuptools import find_packages, setup

package_name = 'mocap4r2_vel_grav_listener'

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
    maintainer='martina',
    maintainer_email='mtahtadadourian@udesa.edu.ar',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vel_grav_listener = mocap4r2_vel_grav_listener.vel_grav_listener:main'
        ],
    },
)
