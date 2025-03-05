from setuptools import setup

package_name = 'model_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'torch', 'numpy'],  # 添加您的依赖项
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='ROS 2 package for model node, processes sensor data',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_node = model_node.main:main',  # 进入点
        ],
    },
)
