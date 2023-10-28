import os
from glob import glob
from setuptools import setup

package_name = 'adrc_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.[pxy][yma]*'))        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DaiGuard',
    maintainer_email='dai_guard@gmail.com',
    description='auto drive rc package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_capture = adrc_ros.data_capture:main',
            'live_run = adrc_ros.live_run:main'
        ],
    },
)
