from setuptools import find_packages, setup
from glob import glob

package_name = 'neuromorphic_collision_avoidance'


# do not remove these links
data_files=[
 ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
 ('share/' + package_name, ['package.xml'])]

# add here extra files and directories so that they are installed in the /install folder and can be available to other packages
data_files += [
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/' + package_name, glob(package_name +'/*.py')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', ]),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gregorio',
    maintainer_email='gremar@kth.se',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    # it is essential to include the entry points so that the scripts can be executed from the command line.
    entry_points={
        'console_scripts': [
            'velocity_controller.py = neuromorphic_collision_avoidance.velocity_controller:main'
        ],
    },
)


