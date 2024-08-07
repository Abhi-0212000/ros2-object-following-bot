from setuptools import find_packages, setup

package_name = 'camera_pkg'

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
    maintainer='abhi',
    maintainer_email='nannuriabhi2000@gmail.com',
    description='Camera Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_node = camera_pkg.camera:main",       # adding the camera node i.e "installer_name = pkg_name.py_filename:main"
            "object_detection_node = camera_pkg.object_detection:main"
        ],
    },
)
