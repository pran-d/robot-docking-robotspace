from setuptools import setup
import os
from glob import glob

package_name = 'my_bot'

def generate_data_files(share_path, dir_name):
    data_files = []
    for root, dirs, files in os.walk(dir_name):
        install_path = os.path.join(share_path, root)
        source_files = [os.path.join(root, f) for f in files]
        data_files.append((install_path, source_files))
    return data_files

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
]

# Add models recursively
data_files.extend(generate_data_files(os.path.join('share', package_name), 'models'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranav',
    maintainer_email='ed21b046@smail.iitm.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = my_bot.aruco_detector:main',
            'docking_node = my_bot.docking_node:main',
        ],
    },
)
