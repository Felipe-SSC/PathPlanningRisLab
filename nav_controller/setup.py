from setuptools import setup

package_name = 'nav_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'manhattanA = nav_controller.manhattanAstar:main',
            'euclideanA = nav_controller.euclideanAstar:main',
            'djikstra = nav_controller.dijkstra:main',
            'meta = nav_controller.meta:main',
            'inicio = nav_controller.inicio:main',
            'square = nav_controller.cuadradoBasic:main',
            'odom = nav_controller.odometry:main',
        ],
    },
)
