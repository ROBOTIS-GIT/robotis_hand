from setuptools import find_packages, setup

package_name = 'robotis_hand_teleop'

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
    maintainer='nhw',
    maintainer_email='nhw@robotis.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_joint_state_publisher = robotis_hand_teleop.hand_joint_state_publisher:main',
            'test_publisher = robotis_hand_teleop.test_publisher:main',
            'vr_publisher = robotis_hand_teleop.vr_publisher:main'
        ],
    },
)
