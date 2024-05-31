from setuptools import find_packages, setup

package_name = 'realsense_camera'

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
    maintainer='billee',
    maintainer_email='billee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = realsense_camera.realsense_publisher:main',
            'uncompress = realsense_camera.uncompress_image:main',
            'calibration = realsense_camera.imu_calibration:main',
            'depth = realsense_camera.pointcloud_publisher:main',
            'server = realsense_camera.image_server:main',
            'client = realsense_camera.camera_client:main',
            'uv_client = realsense_camera.uv_client:main',
            'test = realsense_camera.test_publisher:main'
        ],
    },
)
