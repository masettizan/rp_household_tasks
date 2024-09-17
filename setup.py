from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rp_household_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hello-robot',
    maintainer_email='masettizannini.a@northeastern.edu',
    description='TODO: Package description',
    license='Apache License 2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gather_data = rp_household_tasks.gather_data:main',
            'turn_to_frame = rp_household_tasks.turn_to_frame:main',
            'client = rp_household_tasks.test_client:main'
        ],
    },
)
