from setuptools import find_packages, setup

package_name = 'meSch_base'

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
    maintainer='kalebbennaveed',
    maintainer_email='kalebbennaveed@gmail.com',
    description='meSch_base',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'meSch_central_node = meSch_base.meSch_base_test:main',

        ],
    },
)
