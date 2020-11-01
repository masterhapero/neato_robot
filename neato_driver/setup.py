from setuptools import setup

package_name = 'neato_driver'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Nugen',
    maintainer_email='jnugen@gmail.com',
    description='This is a generic driver for the Neato Botvac Robotic Vacuums.',
    license='BSD',
    entry_points={
        'console_scripts': [
        ],
    },
)
