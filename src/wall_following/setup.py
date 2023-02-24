from setuptools import setup

package_name = 'wall_following'

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
    maintainer='nan',
    maintainer_email='to@do.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sing_vehicle_mode = wall_following.single_vehicle:main',
            'head_to_head_mode = wall_following.head_to_head:main'
        ],
    },
)
