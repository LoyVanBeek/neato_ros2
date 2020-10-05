from setuptools import setup

package_name = 'neato_ros2_python'

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
    maintainer='loy',
    maintainer_email='loy.vanbeek@gmail.com',
    description='Interface to Neato vacuumrobots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neato_node = neato_ros2_python.neato_node:main',
            'neato_driver = neato_ros2_python.neato_driver:main'
        ],
    },
)
