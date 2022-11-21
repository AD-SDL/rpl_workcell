from setuptools import setup
import os
from glob import glob

package_name = 'pf400_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))  
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Doga Ozgulbas, Rafael Vescovi',
    maintainer_email='dozgulbas@anl.gov, ravescovi@anl.gov',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pf400_description_client = pf400_description.pf400_description_client:main'
        ],
    },
)