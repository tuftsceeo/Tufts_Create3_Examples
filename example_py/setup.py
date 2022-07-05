from setuptools import setup

package_name = 'example_py'

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
        'console_scripts': ['test_executable = example_py.pub_LED:main', 'test_executable_2 = example_py.bump_color_rotate:main', 'test_executable_3 = example_py.wheel_chime:main','mid_summer_demo = example_py.mid_summer_demo:main'
        ],
    },
)
