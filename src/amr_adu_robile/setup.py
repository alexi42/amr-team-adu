""""""
from setuptools import find_packages, setup

package_name = 'amr_adu_robile'

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
    maintainer='alexi',
    maintainer_email='alexandra.zarkh@smail.inf.h-brs.de',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follower = amr_adu_robile.wall_follower:main',
            'potential = amr_adu_robile.pot_field_path_planner:main'
        ],
    },
)
