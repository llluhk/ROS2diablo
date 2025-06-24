from setuptools import setup

package_name = 'model_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
        
    data_files=[
        # ...
        ('share/' + package_name + '/checkpoints', [
                'checkpoints/mlp.pkl',
                'checkpoints/scaler.pkl',
        ]),
    ],
    
    install_requires=['setuptools', 'numpy'],  # Your dependencies
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='ROS 2 package for model node, processes sensor data',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_node = model_node.main:main',  # Your executable entry point
        ],
    },
)
