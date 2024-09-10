from setuptools import find_packages, setup

package_name = 'rp_household_tasks'

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
    maintainer='hello-robot',
    maintainer_email='masettizannini.a@northeastern.edu',
    description='TODO: Package description',
    license='Apache License 2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gather_data = rp_household_tasks.gather_data:main',
            'test = rp_household_tasks.test:main'
        ],
    },
)
