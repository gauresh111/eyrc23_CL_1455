from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'mani_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
(os.path.join('lib', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('lib', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('lib', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
        (os.path.join('lib', package_name, 'assets'), glob(os.path.join('assets', '*.stl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gauresh',
    maintainer_email='gauresh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_script = my_package.my_script:main'
        ],
    },
)
