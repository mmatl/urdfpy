"""
Author: Matthew Matl
"""
from setuptools import setup

requirements = [
    'lxml',
    'networkx',
    'numpy',
    'six',
    'trimesh',
]

exec(open('urdfpy/version.py').read())

setup(
    name='urdfpy',
    version = __version__,
    description = 'URDF parser and viewer for Python',
    long_description = 'URDF parser and viewer for Python',
    author = 'Matthew Matl',
    author_email = 'matthewcmatl@gmail.com',
    license = 'MIT License',
    url = 'https://github.com/mmatl/urdfpy',
    keywords = 'robotics ros urdf robots',
    classifiers = [
        'Development Status :: 4 - Beta',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Natural Language :: English',
        'Topic :: Scientific/Engineering'
    ],
    packages = ['urdfpy'],
    install_requires = requirements,
)
