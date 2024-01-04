from setuptools import setup

package_name = 'r-gator-gazebo-2'  # Change this to your package name

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
    maintainer='Your Name',  # Change this to your name
    maintainer_email='your.email@example.com',  # Change this to your email
    description='Description of your package',  # Provide a brief description
    license='License of your choice',  # Specify the license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # List your nodes here as 'executable_name = package.module:main'
            # For example, if your node script is 'r_gator_control.py' in the 'r_gator_gazebo_2' module:
            'r_gator_control = r-gator-gazebo-2.r_gator_control:main',
            # Add other nodes here
        ],
    },
)
