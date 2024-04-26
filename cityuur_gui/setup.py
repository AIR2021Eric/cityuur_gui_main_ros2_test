from setuptools import setup

package_name = 'cityuur_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'sensor_msgs'],
    zip_safe=True,
    maintainer='jacky',
    maintainer_email='jacky@todo.todo',
    description='The cityuur_gui package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'cityuur_gui = cityuur_gui.cityuur_gui:main',
        ],
    },
)
