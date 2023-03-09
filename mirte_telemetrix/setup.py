from setuptools import setup
from glob import glob

package_name = 'mirte_telemetrix'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Klomp',
    maintainer_email='m.klomp@tudelft.nl',
    description='TODO: Package description',
    license='AGPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mirte_telemetrix = mirte_telemetrix.ROS_telemetrix_api:main',
        ],
    },
)
