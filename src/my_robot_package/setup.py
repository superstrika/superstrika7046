from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # This must match the folder name
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Button press ROS 2 node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'test1 = my_robot_package.Neopixel.py',
            'test2 = my_robot_package.ultra_screen:main'
        ],
    },
)
