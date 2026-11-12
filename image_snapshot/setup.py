from setuptools import find_packages, setup

package_name = 'image_snapshot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thalesasoares',
    maintainer_email='thalesasoares02@gmail.com',
    description='TakePhoto service for any image topic',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_snapshot = image_snapshot.node:main',
        ],
    },
)
