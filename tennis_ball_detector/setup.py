from setuptools import setup

package_name = 'tennis_ball_detector'
submodules = 'tennis_ball_detector/scripts'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='estellearrc',
    maintainer_email='estelle.arricau@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = tennis_ball_detector.detection_node:main'
        ],
    },
)
