from setuptools import setup, find_packages

package_name = 'task_allocation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        include=['task_allocation', 'task_allocation.apso_core']
    ),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/full_robot.launch.py']),
        ('share/' + package_name + '/launch', ['launch/apso_test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apso_task_allocation=task_allocation.apso_task_allocation:main',
            'sim_positions=task_allocation.sim_positions:main',
            'robot_sim=task_allocation.robot_sim:main',
            'sim_tasks=task_allocation.sim_tasks:main',
            'simple_2d_viz=task_allocation.simple_2d_viz:main',
            'fake_camera_server=task_allocation.fake_camera_server:main',
        ],
    },
)
