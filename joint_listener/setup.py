from glob import glob
from setuptools import find_packages, setup

setup(
    name='joint_listener',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/joint_listener']),
        ('share/joint_listener', ['package.xml']),
        ('share/joint_listener/config', glob('config/*.yaml')),
        ('share/joint_listener/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='niklas',
    maintainer_email='terei@match.uni-hannover.de',
    description='Invalidate component measurements after configured robot joints move.',
    license='TODO: License declaration',
    entry_points={'console_scripts': [
        'joint_listener = joint_listener.node:main',
    ]},
)
