from setuptools import find_packages, setup

package_name = 'mocap4r2_rigid_body_listener'

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
            'rigid_bodies_listener = mocap4r2_rigid_body_listener.rigid_bodies_listener:main'
        ],
    },
)
