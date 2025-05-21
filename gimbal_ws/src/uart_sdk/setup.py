from setuptools import find_packages, setup

package_name = 'uart_sdk'

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
    maintainer='xiaol',
    maintainer_email='xiaol@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest', 'rclpy'],
    entry_points={
        'console_scripts': [
    'set_position = uart_sdk.set_position:main'
        
        ],
    },
)
