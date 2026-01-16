from setuptools import find_packages, setup
from glob import glob
package_name = 'dm_robotarm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/urdf', glob("urdf/*")), 
        ('share/' + package_name+'/meshes', glob("meshes/*")), 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='an',
    maintainer_email='14626074+ahaoran@user.noreply.gitee.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
