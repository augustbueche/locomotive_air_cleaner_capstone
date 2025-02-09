from setuptools import setup

package_name = 'ultrasonic_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amhunpdx',
    description='Publishes ultrasonic sensor data from ESP32 via serial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_publisher = ultrasonic_publisher.publisher:main',
            'ultrasonic_subscriber = ultrasonic_publisher.subscriber:main',
        ],
    },
)
