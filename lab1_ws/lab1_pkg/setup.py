from setuptools import setup
package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lab1_launch.py']), # Add Launch file to Datas
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='patchy',
    maintainer_email='patchy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ # Make sure to add all nodes here
            'talker = lab1_pkg.talker:main',
            'relay = lab1_pkg.relay:main',
        ],
    },
)