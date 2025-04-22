from setuptools import setup
import os
from glob import glob

package_name = 'qube_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='jon Håvard Sylte',
    maintainer_email='jhsylte@stud.ntnu.no',
    description='PID controller for Qube',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qube_controller = qube_controller.controller_node:main',
        ],
    },
)
