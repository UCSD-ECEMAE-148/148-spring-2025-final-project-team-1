from setuptools import setup

package_name = 'autonomous_tow_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/tow_sequence.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='George Nakoud',
    maintainer_email='gnakoud@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'detect_damsel = autonomous_tow_car.detect_damsel:main',
        'three_point_turn = autonomous_tow_car.three_point_turn:main',
        'hook_and_vanish = autonomous_tow_car.hook_and_vanish:main',
	],
    },
)
