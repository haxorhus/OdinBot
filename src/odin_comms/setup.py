from setuptools import find_packages, setup

package_name = 'odin_comms'

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
    maintainer='jose',
    maintainer_email='joseferreira@fiuna.edu.py',
    description='En este paquete se encuentran todos los nodos que comunican la PC con ROS con el robot fisico',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_mr = odin_comms.control_motor1:main',
            'control_ml = odin_comms.control_motor2:main',
            'data_subscriber = odin_comms.data_subscriber:main',
            'control_PID = odin_comms.control_PID:main',
            'control_twist = odin_comms.control_twist:main'
        ],
    },
)
