from setuptools import find_packages
from setuptools import setup

package_name = 'robotis_hand_teleop'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    author='Howon Kim',
    author_email='rlaghdnjs17@naver.com',
    description='Robotis Hand keyboard teleoperation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hx5_d20_right_teleop = robotis_hand_teleop.hx5_d20_right_teleop:main',
            'hx5_d20_left_teleop = robotis_hand_teleop.hx5_d20_left_teleop:main'
        ],
    },
)
