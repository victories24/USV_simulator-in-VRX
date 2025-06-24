from setuptools import find_packages, setup

package_name = 'mywamv_control'

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
    maintainer='cj',
    maintainer_email='cj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
    scripts=['scripts/mywamv_inverse_kinematics.py',
    'scripts/mywamv_path_follow.py',
    'scripts/mywamv_station_keeping.py',
    'scripts/mywamv_wayfinding.py',
    'scripts/dubins_path_generator.py',
    'scripts/figure_eight_generator.py'],
)
