from setuptools import setup
import os
from glob import glob

package_name = 'nav2_indoor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rosa Maria Hernandez',
    maintainer_email='rmherval@upv.edu.es',
    description='Description package for the FW Mini robot (URDF, meshes, and launch files)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'pc_to_laser = nav2_indoor.pc_to_laser:main'
        ],
    },
)

