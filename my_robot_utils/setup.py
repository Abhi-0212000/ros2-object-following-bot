from setuptools import setup

package_name = 'my_robot_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},  # This maps the 'src' directory to the package
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),  # Ensure this line is correct
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package for helper functions and utilities',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
