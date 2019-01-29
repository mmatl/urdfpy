"""
Author: Matthew Matl
"""
from setuptools import setup

requirements = [
]

#exec(open('ambidex/version.py').read())


setup(
    name='urdfpy',
    #version = __version__,
    #description = 'Dex-Net',
    #long_description = 'Core utilities for the Berkeley AutoLab. Includes rigid transformations, loggers, and 3D data wrappers.',
    #author = 'Matthew Matl',
    #author_email = 'matthewcmatl@gmail.com',
    #license = 'Apache Software License',
    #url = 'https://github.com/BerkeleyAutomation/autolab_core',
    #keywords = 'robotics grasping transformations',
    #classifiers = [
    #    'Development Status :: 4 - Beta',
    #    'License :: OSI Approved :: Apache Software License',
    #    'Programming Language :: Python',
    #    'Programming Language :: Python :: 2.7',
    #    'Programming Language :: Python :: 3.5',
    #    'Programming Language :: Python :: 3.6',
    #    'Natural Language :: English',
    #    'Topic :: Scientific/Engineering'
    #],
    packages = ['urdfpy'],
    install_requires = requirements,
)


