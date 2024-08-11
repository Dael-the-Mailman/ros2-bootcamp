from setuptools import find_packages, setup

package_name = 'turtlebot_random'

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
    maintainer='dev-desktop',
    maintainer_email='kalebugalde@gmail.com',
    description='Move turtlebot to a random goal',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start = turtlebot_random.random_goal:main'
        ],
    },
)
