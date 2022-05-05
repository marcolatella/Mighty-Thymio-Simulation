from setuptools import setup
from glob import glob


package_name = 'thymio_ml'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usi',
    maintainer_email='usi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = thymio_ml.controller_node:main',
            'controller_node_8 = thymio_ml.controller_node_8:main',
            'bonus_node = thymio_ml.bonus_node:main'
        ],
    },
)
