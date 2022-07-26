from setuptools import setup

package_name = 'integration'
PPO = "integration/PPO"
CA = "integration/CollisionAvoidance/ArtificialPotentialField"
PP = "integration/PathPlanning/RRT"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, PPO, CA, PP],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='foxy',
    maintainer_email='foxy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'IntegrationTest = integration.integration_offboard:main'
        ],
    },
)
