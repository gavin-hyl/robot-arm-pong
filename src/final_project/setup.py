from setuptools import find_packages, setup
from glob import glob

package_name = 'final_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gavin Yuliu Hua',
    maintainer_email='ghua@caltech.edu',
    description='Final project for ME/EE/CS 133a - Robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory = final_project.Trajectory:main',
            'ball_demo = final_project.Balldemo:main',
        ],
    },
)
