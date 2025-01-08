from setuptools import setup

package_name = 'my_car_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.17.3,<1.25.0',  # Specific version range for SciPy compatibility
        'scipy',
        'scikit-learn',
    ],
    zip_safe=True,
    maintainer='disney',
    maintainer_email='disney@todo.todo',
    description='Autonomous vehicle controller with RRT* planning',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_controller = my_car_controller.car_controller:main'
        ],
    },
)