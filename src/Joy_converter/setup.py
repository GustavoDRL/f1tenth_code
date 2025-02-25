from setuptools import setup
from glob import glob
import os

package_name = 'joy_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='disney',
    maintainer_email='gustavo.rio@aluno.ufabc.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_ackerman = joy_converter.joy_ackerman:main',
            'joy_twist = joy_converter.joy_twist:main',
        ],
    },
)
