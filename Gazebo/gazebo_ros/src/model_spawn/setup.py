from setuptools import setup

package_name = 'model_spawn'
# model_spawn_srvs = './model_spawn_srvs'

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
    maintainer='foxy',
    maintainer_email='dmxwk98@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ModelSpawn = model_spawn.model_spawn:main'
        ],
    },
)
