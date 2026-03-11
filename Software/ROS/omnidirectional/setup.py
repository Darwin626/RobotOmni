from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'omnidirectional'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch/'), 
            glob('launch/*launch.[pxy][yma]*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simple = omnidirectional.simple_node:main",
            "inversa = omnidirectional.cin_inv:main",
            "odome  = omnidirectional.rv:main",
            "w1     = omnidirectional.ser_read_w1:main",
            "w2     = omnidirectional.ser_read_w2:main",
            "w3     = omnidirectional.ser_read_w3:main",
            "odome_imu = omnidirectional.odom_imu:main",
            "odom_encoder_gz = omnidirectional.odom_encoder_gz:main",
            "odom_gt = omnidirectional.odom_gt_gz:main",
            "sensor = omnidirectional.sensor_read:main"
            
        ],
    },
)
