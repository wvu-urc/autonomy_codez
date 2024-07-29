from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'autonomy_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))) # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='nathanpadkins@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomy_ptp_planner_node = autonomy_pkg.planner:main',
            'autonomy_text_interface_node = autonomy_pkg.interface:main',
            'object_chaser_node = autonomy_pkg.object_chaser:main',
            'waypoint_queue_node = autonomy_pkg.waypoint_queue:main',
        ],
    },
)
