from setuptools import find_packages, setup

package_name = 'vacuum_turtle'

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
    maintainer='Kaleb Ugalde',
    maintainer_email='kalebugalde@gmail.com',
    description='Have the turtle go in a spiral',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start = vacuum_turtle.spiral_turtle:main',
        ],
    },
)
