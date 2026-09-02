from setuptools import find_packages, setup

package_name = 'catchrobo_joy_test'

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
    maintainer='ubuntu',
    maintainer_email='kazumaokamoto1105@gmail.com',
    description='joy_test: Package description',
    license='joy_test: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_sub_test = catchrobo_joy_test.joy_subscriber:main',
        ],
    },
)
