from setuptools import setup
import os
from glob import glob

package_name = 'camera_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'id_detection'), glob('id_detection/*.blob')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_executable = camera_control.camera_control:main',
            'aruco_detection_executable = aruco_marker.aruco_detection:main',
            'person_detection_executable = id_detection.person_ident:main',
            'counter_publisher = aruco_marker.counter_publisher:main',
        ],
    },
)
