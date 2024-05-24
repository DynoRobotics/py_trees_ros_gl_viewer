from setuptools import find_packages, setup

package_name = 'py_trees_ros_gl_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Erik Örjehag',
    maintainer_email='',
    description='Alternative py trees ros viewer, with high FPS for large trees',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py-trees-ros-gl-viewer = py_trees_ros_gl_viewer.viewer:main'
        ],
    },
)
