from setuptools import setup
# from setuptools import find_packages
from glob import glob

package_name = 'thymio_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/aseba', glob('aseba/*.aesl')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
        ('share/' + package_name + '/calibration', glob('calibration/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    keywords=['Thymio'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A ROS driver for the Thymio-II robot',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thymio_driver = thymio_driver.thymio_driver_node:main',
            'multi_thymio_driver = thymio_driver.multi_thymio_driver_node:main',
            'follow_square = thymio_driver.follow_square:main',
            'calibrate_motors = thymio_driver.calibrate_motors:main',
        ],
    },
)
