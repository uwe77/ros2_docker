from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'test_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Beginner client libraries tutorials practice python package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = test_py_pkg.publisher_member_function:main',
            'listener = test_py_pkg.subscriber_member_function:main',
            'service = test_py_pkg.service_member_function:main',
            'client = test_py_pkg.client_member_function:main',
            'minimal_param_node = test_py_pkg.python_parameters_node:main',
        ],
    },
)
