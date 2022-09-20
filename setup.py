import os
from glob import glob

from setuptools import setup

package_name = 'solo12_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob('config/*config.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include all config files.
        (os.path.join('share', package_name, 'ressource'), glob('ressource/*.np[y][z]'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugo',
    maintainer_email='29019005+Bacmel@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solo12_viewer = solo12_viewer.solo12_viewer:main'
        ],
    },
)
