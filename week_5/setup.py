from glob import glob

from setuptools import find_packages, setup

package_name = 'week_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/models/week_5_world', glob('models/week_5_world/*')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pedro Ribeiro',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_fault = week_5.lidar_fault:main',
            'robot_controller = week_5.robot_controller:main'
        ],
    },
)
