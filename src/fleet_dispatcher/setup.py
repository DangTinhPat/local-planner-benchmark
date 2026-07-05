from setuptools import find_packages, setup

package_name = 'fleet_dispatcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dvt',
    maintainer_email='dangtinh.ftcpy@gmail.com',
    description='Central dispatcher assigning warehouse putaway jobs to idle robots',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_dispatcher = fleet_dispatcher.dispatcher:main',
        ],
    },
)
