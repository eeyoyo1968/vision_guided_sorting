from setuptools import setup

package_name = 'grasp_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raico',
    maintainer_email='raico@todo.todo',
    description='Grasp perception ROS2 node (wraps YOLO pipeline)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'grasp_node = grasp_perception.grasp_node:main',
        'grasp_node2 = grasp_perception.grasp_node2:main',
        'grasp_node3 = grasp_perception.grasp_node3:main',
        'grasp_node4 = grasp_perception.grasp_node4:main',
        'grasp_node5 = grasp_perception.grasp_node5:main',
        'grasp_node6 = grasp_perception.grasp_node6:main',
    ],
},
)

