from setuptools import find_packages, setup

package_name = 'my_python_pkg'

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
    maintainer='vertin',
    maintainer_email='vertin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autopoint = my_python_pkg.autopoint:main',
            'maptoodom = my_python_pkg.map_odom:main',
        ],
    },
)
