from setuptools import setup

package_name = 'group'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch directory
        ('share/' + package_name + '/launch', ['launch/webots_launch.py']),
        # Include the worlds directory
        ('share/' + package_name + '/worlds', ['worlds/my_world.wbt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@domain.com',
    description='Python package to launch Webots with a custom world',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'], 
        'console_scripts': ['controller = group.group:main']
            # You can add Python node entry points here if needed
    
    },
)

