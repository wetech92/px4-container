from setuptools import setup

package_name = 'g_camera_lidar_sp'

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
    maintainer='ACSL',
    maintainer_email='iacslsyno@gmail.com',
    description='TODO: Package description',
    license='BSD-3 Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "all = g_camera_lidar_sp.all:main",
            "lidar = g_camera_lidar_sp.lidar:main",
            "camera = g_camera_lidar_sp.camera:main"
        ],
    },
)
