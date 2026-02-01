from setuptools import setup

package_name = 'odometry_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    'setuptools',
    'tf-transformations',],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Differential drive odometry node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry = odometry_pkg.odometry:main',
        ],
    },
)
