import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mujina_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RT Corporation',
    maintainer_email='shop@rt-net.jp',
    description='MUJINA control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujina_main = mujina_control.mujina_main:main',
        ],
    },
)
