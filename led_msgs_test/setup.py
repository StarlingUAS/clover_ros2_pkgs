from setuptools import setup

package_name = 'led_msgs_test'

setup(
    name=package_name,
    version='0.0.9',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oleg Kalachev',
    maintainer_email='rokalachev@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim = led_msgs_test.sim:main'
        ],
    },
)
