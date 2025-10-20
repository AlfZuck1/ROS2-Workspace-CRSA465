from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'CRSA465'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Incluir archivos de launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Incluir archivos URDF
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Incluir meshes y texturas
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'textures'), glob('textures/*')),
        # Incluir configuración (como rviz)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zuck',
    maintainer_email='zuck@todo.todo',
    description='Modelo URDF del robot CRSA465',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = CRSA465.control_node:main'
        ],
    },
)
