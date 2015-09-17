__author__ = 'zmbq'

import numpy as np
cimport numpy as np
cimport tetgen
from libcpp cimport bool
from libc.string cimport memcpy

# Numpy declarations
ctypedef np.ndarray[np.float64_t, ndim=2] PointArray

cdef init_input(PointArray &pts, PointArray &big_tetrahedron, tetgen.tetgenio &input):
    input.first_number = 0
    input.numberofpoints = 4 + pts.shape[0]
    input.pointlist = new double[in.numberofpoints * 3]

    # Copy the data directly
    memcpy(input.pointlist, pts.data, pts.shape[0] * 3 * sizeof(double))
    memcpy(input.pointlist + pts.shape[0] * 3, big_tetrahedron.data, 4 * 3 * sizeof(double))

cdef call_tetgen(tetgen.genio &input, tetgen.genio &output, bool run_voronoi):
    flags = 'nQv' if run_voronoi else 'nQ'
    tetgen.tetrahedralize(flags.encode('UTF-8'), &input, &output)

def run(PointArray pts, PointArray big_tetrahedron, bool run_voronoi):
    if pts.shape[1]!=3:
        raise ValueError("pts must be an array of 3D vectors")
    if big_tetrahedron.shape!=(4,3):
        raise ValueError("big_tetrahedron must contain 4 3D vectors")
    if not pts.flags['C_CONTIGUOUS']:
        raise ValueError("pts must be C-contiguous")
    if not big_tetrahedron.flags['C_CONTIGUOUS']:
        raise ValueError("big_tetrahedron must be C_CONTIGUOUS")

    cdef tetgen.tetgenio input, output
    init_input(pts, big_tetrahedron, input)
    call_tetgen(input, output, run_voronoi)


    # No need to free anything, the destructors will take care of that