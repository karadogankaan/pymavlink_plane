from setuptools import find_packages, setup

package_name = 'kaan_deneme'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name), glob('launch/launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaan',
    maintainer_email='kaan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'kaan1 = kaan_deneme.mov1.py:main',
        'kaan2 = kaan_deneme.mov2.py:main',
        'kaan3 = kaan_deneme.mov3.py:main'
        ],
    },
)
