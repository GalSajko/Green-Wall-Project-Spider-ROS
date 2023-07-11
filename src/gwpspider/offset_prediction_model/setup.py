from setuptools import setup
import os
from glob import glob

package_name = 'offset_prediction_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.pth'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spider',
    maintainer_email='gal.sajko@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offset_predictor = offset_prediction_model.offset_predictor:main'
        ],
    },
)
