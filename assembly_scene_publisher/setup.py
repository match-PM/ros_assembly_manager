from setuptools import find_packages, setup

package_name = 'assembly_scene_publisher'
submodules = 'assembly_scene_publisher/py_modules'

setup(
    name=package_name,
    version='0.0.0',
    #packages=[find_packages(exclude=['test'])],
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niklas',
    maintainer_email='terei@match.uni-hannover.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'assembly_scene_publisher = assembly_scene_publisher.assembly_scene_publisher:main',
        ],
    },
)
