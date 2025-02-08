from setuptools import find_packages, setup

package_name = 'mujoco_task_planner'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages = [package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hs_dyros',
    maintainer_email='hs_dyros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        "test": ["pytest"]},
    entry_points={
        'console_scripts': [
            'block_sorting = mujoco_task_planner.block_sorting:main',
            'peg_in_hole = mujoco_task_planner.peg_in_hole:main',
            'server = mujoco_task_planner.test_server:main',
        ],
    },
)
