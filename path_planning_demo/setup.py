from setuptools import find_packages, setup

package_name = 'path_planning_demo'

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
    maintainer='Abhishek Sankar',
    maintainer_email='241abhishek@gmail.com',
    description='Cartesian path planning demo for a modified ur10e robotic arm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cartesian_path_plan = path_planning_demo.cartesian_path_plan:main",
            "ee_velocity = path_planning_demo.ee_velocity:main",
        ],
    },
)
