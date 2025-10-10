from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'moveit_ur10'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'sim'), glob('sim/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Faran Akram',
    maintainer_email='faran1218@gmail.com',
    description='An automatically generated package with all the configuration and launch files for using the ur10_with_gripper with the MoveIt Motion Planning Framework',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik = scripts.ik:main',
            'my_moveit = scripts.my_moveit:main',
            'simple_moveit = scripts.simple_moveit:main',
        ],
    },
)
