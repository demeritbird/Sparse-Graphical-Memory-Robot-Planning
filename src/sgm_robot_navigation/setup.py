from setuptools import find_packages, setup

package_name = 'sgm_robot_navigation'

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
    maintainer='demeritbird',
    maintainer_email='lekjiewei@hotmail.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_controller_server = sgm_robot_navigation.robot_controller_server:main",
            "marker_points_node = sgm_robot_navigation.marker_points_node:main"
        ],
    },
)
