from setuptools import setup
import os
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suho',
    maintainer_email='tnghdjaak75@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flask_node = my_package.flask_node:main',
            'sensor_node = my_package.sensor_node:main',
            'monitor_node = my_package.monitor_node:main',
            'control_node = my_package.control_node:main',
            'motor_node = my_package.motor_node:main',
        ],
    },
)
