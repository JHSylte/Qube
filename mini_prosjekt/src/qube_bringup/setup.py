from setuptools import find_packages, setup

package_name = 'qube_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/controlled_qube.urdf.xacro']),
        ('share/' + package_name + '/rviz', ['rviz/view_qube.rviz']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tryvann',
    maintainer_email='trymva@stud.ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
