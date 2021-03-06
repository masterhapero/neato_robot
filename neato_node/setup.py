from setuptools import setup
import os
from glob import glob

package_name = 'neato_node'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Nugen',
    maintainer_email='jnugen@gmail.com',
    description='A node wrapper for the neato_driver package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neato_node = neato_node.neato_node:main'
        ],
    },
)
