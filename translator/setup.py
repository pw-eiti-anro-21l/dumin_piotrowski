from setuptools import setup

package_name = 'translator'

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
    maintainer='Mateusz Dumin',
    maintainer_email='mateusz.dumin.stud@pw.edu.pl',
    description='Simple package for controlling turtlesim from keybord',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = translator.KeyLogger:main',
                'listener = translator.publisher_member_function:main',
        ],
    },
)
