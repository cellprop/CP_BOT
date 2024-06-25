from setuptools import setup

package_name = 'CP_BOT'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim',
    maintainer_email='vihaan@cellpropulsion.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_robot_controller = CP_BOT.mqtt_robot_controller:main',
            'road_follower = CP_BOT.road_follower:main',
        ],
    },
)
