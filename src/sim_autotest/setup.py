from setuptools import setup
package_name = 'sim_autotest'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Gazebo auto simulation + tracker test harness',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'spawner = sim_autotest.spawner:main',
            'mover = sim_autotest.mover:main',
            'detector = sim_autotest.detector:main',
            'evaluator = sim_autotest.evaluator:main',
        ],
    },
)
