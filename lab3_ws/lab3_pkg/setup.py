from setuptools import setup

package_name = 'lab3_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lab3_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='patchy',
    maintainer_email='patchy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PID = lab3_pkg.PID:main',
        ],
    },
)
