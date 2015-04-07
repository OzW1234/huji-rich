/*! \file hdf5_diagnostics.hpp
\brief Simulation output to hdf5 file format for 3D simulation
\author Elad Steinberg
*/

#ifndef HDF5_DIAG
#define HDF5_DIAG 1

#include <H5Cpp.h>
#include <string>
#include "hdsim_3d.hpp"

/*!
\brief Writes the simulation data into an HDF5 file
\param sim The hdsim class of the simulation
\param fname The name of the output file
*/
void write_snapshot_to_hdf5(HDSim3D const& sim, string const& fname);

#endif