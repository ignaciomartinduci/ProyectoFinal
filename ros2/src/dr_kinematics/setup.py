from setuptools import find_packages, setup

package_name = 'dr_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ignaciomartind',
    maintainer_email='ignaciomartind@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_node = dr_kinematics.nodes.ik_node:main',
            'fk_node = dr_kinematics.nodes.fk_node:main'
        ],
    },
)
