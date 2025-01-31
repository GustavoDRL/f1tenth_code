from setuptools import setup
import os
from glob import glob

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='disney',
    maintainer_email='gustavo.rio@aluno.ufabc.edu.br',
    description='Gap following navigation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_follow = gap_follow.gap_follow:main',
            'pid_visualizer = gap_follow.pid_visualizer:main',
        ],
    },
)