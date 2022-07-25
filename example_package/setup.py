from setuptools import setup

package_name = 'example_package'

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
    maintainer='maddiepero',
    maintainer_email='maddie.pero@tuftsceeo.org',
    description='Test Package for Project Create',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['subscribe = example_package.sub_ir:main','publish = example_package.pub_lightring:main', 'action = example_package.action_drive_square:main', 'action2 = example_package.action_drive_square_2:main', 'combined = example_package.combined_audio_bump:main',
                            'dock = example_package.action_dock:main', 'undock = example_package.action_undock:main', 'tcp=example_package.tcp_node:main'
        ],
    },
)
