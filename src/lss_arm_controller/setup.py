from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lss_arm_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Custom controllers for LSS 4DOF Arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = lss_arm_controller.perception_node:main',
            'lamp_adjuster = lss_arm_controller.lamp_adjuster:main',
            'box_mover = lss_arm_controller.box_mover:main',
            'demo_controller = lss_arm_controller.demo_controller:main',
            'joint_commander = lss_arm_controller.joint_commander:main',
            'fake_joint_executor = lss_arm_controller.fake_joint_executor:main',
        ],
    },
)
