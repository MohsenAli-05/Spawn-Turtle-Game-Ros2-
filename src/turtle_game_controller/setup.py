from setuptools import find_packages, setup

package_name = 'turtle_game_controller'

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
    maintainer='mohsenium',
    maintainer_email='mohsenium@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_spawner_node = turtle_game_controller.turtle_spawner:main",
            "turtle_mover_node = turtle_game_controller.turtle_movement_controller:main",
            "turtle_crash_node = turtle_game_controller.turtle_crash_handler:main"
        ],
    },
)
