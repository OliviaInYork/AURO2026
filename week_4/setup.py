from setuptools import find_packages, setup

package_name = 'week_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'task3 = week_4.task3:main',
            'task7 = week_4.task7:main',
            'task8_TurtleBot3FSMRC = week_4.task8_TurtleBot3FSMRC:main'
        ],
    },
)
