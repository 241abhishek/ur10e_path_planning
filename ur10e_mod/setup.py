from setuptools import find_packages, setup

package_name = 'ur10e_mod'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/visualize_ur10e_mod.launch.xml', 'launch/ur10e_mod_sim_control.launch.py']),
        ('share/' + package_name + '/urdf',
            ['urdf/ur10e_mod.urdf.xacro', 'urdf/ur10e_mod_gz.urdf.xacro',
             'urdf/ur10e_mod_gz.ros2_control.xacro','urdf/ur10e_mod_joint_control.xacro']),
        ('share/' + package_name + '/config',
            ['config/view_robot.rviz', 'config/ur_controllers_with_z_lift.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhishek Sankar',
    maintainer_email='241abhishek@gmail.com',
    description='Modified UR10e robot description package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
