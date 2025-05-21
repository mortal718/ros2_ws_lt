from setuptools import find_packages, setup
from glob import glob
import os



package_name = 'topic_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaol',
    maintainer_email='xiaol@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_teleop_omni = topic_pkg.joystick_teleop_omni:main',
            'nav_modal = topic_pkg.nav_modal:main',
            'head_control = topic_pkg.head_control:main',
            'safety_joy_interface = topic_pkg.safety_joy_interface:main',
            'avoidance_controller = topic_pkg.avoidance_controller:main',
        ],
    },
)
