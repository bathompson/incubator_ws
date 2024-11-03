from setuptools import find_packages, setup

package_name = 'incubator_py_pkg'

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
    maintainer='thompson',
    maintainer_email='bathompson@ksu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "Incubator_i_Instance_dt_dtp_pkf_exe = incubator_py_pkg.Incubator_i_Instance_dt_dtp_pkf:main"
        ],
    },
)