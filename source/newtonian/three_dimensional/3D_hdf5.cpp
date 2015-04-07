#include "3D_hdf5.hpp"
#include "../../misc/hdf5_utils.hpp"

using namespace H5;

void write_snapshot_to_hdf5(HDSim3D const& sim, string const& fname)
{
	vector<ComputationalCell> const& cells=sim.getCells();
	vector<double> temp(cells.size());
	HDF5Shortcut h5sc(fname);
	for (size_t i = 0; i < cells.size(); ++i)
		temp[i] = cells[i].density;
	h5sc("Density", temp);
	for (size_t i = 0; i < cells.size(); ++i)
		temp[i] = cells[i].pressure;
	h5sc("Pressure", temp);
	for (size_t i = 0; i < cells.size(); ++i)
		temp[i] = cells[i].velocity.x;
	h5sc("x_velocity", temp);
	for (size_t i = 0; i < cells.size(); ++i)
		temp[i] = cells[i].velocity.y;
	h5sc("y_velocity", temp);
	for (size_t i = 0; i < cells.size(); ++i)
		temp[i] = cells[i].velocity.z;
	h5sc("z_velocity", temp);
	Tessellation3D const& tess=sim.getTesselation();
	for (size_t i = 0; i < cells.size(); ++i)
		temp[i] = tess.GetMeshPoint(i).x;
	h5sc("x_coordinate", temp);
	for (size_t i = 0; i < cells.size(); ++i)
		temp[i] = tess.GetMeshPoint(i).y;
	h5sc("y_coordinate", temp);
	for (size_t i = 0; i < cells.size(); ++i)
		temp[i] = tess.GetMeshPoint(i).z;
	h5sc("z_coordinate", temp);
}