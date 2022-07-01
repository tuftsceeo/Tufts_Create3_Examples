from setuptools import setup

package_name = 'Mini-Examples_Library'

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
    description='Tufts University Mini-Examples for the iRobot® Create® 3 Educational Robot',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['Pub_Docking = Mini-Examples_Library.Pub_Docking:main'
        ],
    },
)
