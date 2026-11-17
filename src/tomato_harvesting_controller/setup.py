from setuptools import find_packages, setup

package_name = 'tomato_harvesting_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    py_modules=['move_to_tomato'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tomato Harvesting Project',
    maintainer_email='maintainer@example.com',
    description='ROS 2 interface for D435 tomato localization and UR10e MoveIt planning.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'move_to_tomato = move_to_tomato:main',
        ],
    },
)
