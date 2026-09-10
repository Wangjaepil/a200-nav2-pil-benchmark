from setuptools import find_packages, setup

package_name = 'dual_gps_heading'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tb-pil',
    maintainer_email='tb-pil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'heading_node = dual_gps_heading.heading_node:main',
            'tf_relay = dual_gps_heading.tf_relay:main',
        ],
    },
)
