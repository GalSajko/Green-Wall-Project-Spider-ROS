from setuptools import setup

package_name = 'connection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'fastapi',
                      'uvicorn',
                      'ros2py',
    ],
    zip_safe=True,
    maintainer='spider',
    maintainer_email='gal.sajko@gmail.com',
    description='Connection layer between the ROS2 environment and the web based GUI',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fastapi = connection.fastapi:main',
            'backend = connection.backend:main'
        ],
    },
)
