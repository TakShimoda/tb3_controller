from setuptools import setup

package_name = 'tb3_controller'

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
    maintainer='glenn',
    maintainer_email='31490076+TakShimoda@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tb3_client = tb3_controller.tb3_client:main",
            "tb3_server = tb3_controller.tb3_server:main",
            "tb3_action_client = tb3_controller.tb3_action_client:main",
            "tb3_action_server = tb3_controller.tb3_action_server:main",
            "tb3_nav_action_server = tb3_controller.tb3_nav_action_server:main",
            "tb3_nav_action_client = tb3_controller.tb3_nav_action_client:main",
            "tb3_PID = tb3_controller.PID_tuning:main",
            "tb3_test = tb3_controller.tb3_velocity_test:main"
        ],
    },
)
