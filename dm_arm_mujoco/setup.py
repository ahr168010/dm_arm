from setuptools import find_packages, setup

package_name = 'dm_arm_mujoco'

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
    maintainer='an',
    maintainer_email='14626074+ahaoran@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "mujoco_show=dm_arm_mujoco.mujoco_show:main",
            "read_moveit_plan=dm_arm_mujoco.read_moveit_plan:main",
            "moveit_mujoco_fjt_server=dm_arm_mujoco.moveit_mujoco_fjt_server:main",
        ],
    },
)
