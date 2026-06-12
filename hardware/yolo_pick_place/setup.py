from setuptools import find_packages, setup

package_name = 'yolo_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_yolo.launch.py']),
        ('share/' + package_name + '/launch', ['launch/webcamtest.launch.py']),
        

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reach',
    maintainer_email='reachsar168@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolotest = yolo_pick_place.yolotest:main',
            'camcal = yolo_pick_place.camcal:main',
            'yolopickplace = yolo_pick_place.yolopickplace:main',
            'resultplot = yolo_pick_place.resultplot:main',
        ],
    },
)













