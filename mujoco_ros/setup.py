from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mujoco_ros'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/mujoco_ros']),  # 패키지 마커 파일
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
]

robots_path = 'robots'
for root, dirs, files in os.walk(robots_path):
    for file in files:
        relative_path = os.path.relpath(root, robots_path)
        install_path = os.path.join('share', package_name, 'robots', relative_path)
        data_files.append((install_path, [os.path.join(root, file)]))

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=data_files,
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
            'mujoco_test = mujoco_ros.test:main',
            'fr3_wo_hand = mujoco_ros.fr3_wo_hand:main',
            'franka_hand = mujoco_ros.franka_hand:main',
            'fr3_w_hand = mujoco_ros.fr3_w_hand:main',
            'fr3_pih = mujoco_ros.fr3_pih:main',

        ],
    },
)
