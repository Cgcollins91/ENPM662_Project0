from setuptools import setup

package_name = 'tb_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cgcollins91',
    maintainer_email='Chris.g.collins1991@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb_openLoop = tb_control.tb_openLoop:main',
            'tb_openLoop_1 = tb_control.tb_openLoop_1:main',
            'pose_subscriber = tb_control.pose_subscriber:main',
        ],
    },
)
