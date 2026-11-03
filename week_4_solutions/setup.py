from glob import glob

from setuptools import find_packages, setup

package_name = 'week_4_solutions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
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
            'task2_match = week_4_solutions.task2_match:main',
            'task2_match_rc = week_4_solutions.task2_match_rc:main',
            'task4_match = week_4_solutions.task4_match:main',
            'task4_match_rc = week_4_solutions.task4_match_rc:main',
            'task5 = week_4_solutions.task5:main',
            'task6 = week_4_solutions.task6:main',
            'task7 = week_4_solutions.task7:main',
            'task8_DetectorFSM = week_4_solutions.task8_DetectorFSM:main',
            'task8_TurtleBot3FSMRC = week_4_solutions.task8_TurtleBot3FSMRC:main',
            'task9 = week_4_solutions.task9:main'
        ],
    },
)
