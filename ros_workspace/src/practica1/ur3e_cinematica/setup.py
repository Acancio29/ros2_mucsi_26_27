from setuptools import find_packages, setup

package_name = 'ur3e_cinematica'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laboratorio',
    maintainer_email='144768058+CarmeloValera@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'suscriptor_articulaciones = ur3e_cinematica.suscriptor_articulaciones:main',
        ],
    },
)
