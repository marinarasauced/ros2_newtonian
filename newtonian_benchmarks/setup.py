
from glob import glob
import xml.etree.ElementTree as ET
from setuptools import find_packages, setup


PACKAGE = 'newtonian_benchmarks'
VERSION = ET.parse('package.xml').getroot().findtext('version')


setup(
    name=PACKAGE,
    version=VERSION,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marina NELSON',
    maintainer_email='marinarasauced@outlook.com',
    description='Robustness benchmark of controllers on the newtonian simulator.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bench = newtonian_benchmarks.node:main',
        ],
    },
)
