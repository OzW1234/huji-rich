__author__ = 'zmbq'

from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
import numpy


TETGEN_DIR = '../../external_libraries/tetgen1.5.0'

setup(
    ext_modules=cythonize(
        [Extension(
            "*",
            ["*.pyx"],
            language='c++',
            include_dirs=[TETGEN_DIR, numpy.get_include()],
            library_dirs=[TETGEN_DIR],
            libraries=['tetgen',],
        )]
    )
)
