from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'eye_expression_pkg'

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
            'eye_expression_node = eye_expression_pkg.eye_expression_node:main',
            'eye_expression_node2 = eye_expression_pkg.eye_expression_node2:main',
            'aaa= eye_expression_pkg.aaa:main',
        ],
    },
)
