from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'assembly_scene_viewer'
submodules = 'assembly_scene_viewer/py_modules'
media = 'assembly_scene_viewer/media'

setup(
    name=package_name,
    version='0.0.0',
    packages=(package_name, submodules),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            (
            os.path.join('share', package_name, 'media'),
            glob('assembly_scene_viewer/media/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mll',
    maintainer_email='terei@match.uni-hannover.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assembly_scene_viewer = assembly_scene_viewer.assembly_scene_viewer:main'
        ],
    },
)
