from setuptools import find_packages, setup

package_name = 'velocity'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'scipy', 'numpy'],
    zip_safe=True,
    maintainer='udesa',
    maintainer_email='tadeo.casiraghi@gmail.com',
    description='Calculates and filters local velocity from mocap rigid bodies',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This links the command name to the python script and main function
            'velocity = velocity.velocity_calc:main'
        ],
    },
)