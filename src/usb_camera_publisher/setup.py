from setuptools import setup

package_name = 'usb_camera_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Publishes multiple USB camera feeds using compressed video transport.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = usb_camera_publisher.publisher:main',
            'image_publisher = usb_camera_publisher.publish_images:main',
            'bev = usb_camera_publisher.bev:main',
            'stitched = usb_camera_publisher.stitched:main',
        ],
    },
)
