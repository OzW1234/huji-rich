#define NO_HDF5

#include <iostream>
#include <string>
#include "../../newtonian/three_dimensional/eulerian_3d.hpp"
#include "../../newtonian/three_dimensional/CourantFriedrichsLewy.hpp"
#include "../../newtonian/three_dimensional/hdsim_3d.hpp"
#include "../../newtonian/three_dimensional/FirstOrderHydroFlux.hpp"

#ifndef NO_HDF5
#include "../../newtonian/three_dimensional/3D_hdf5.hpp"
#endif

#include "../GeometryCommon/OuterBoundary3D.hpp"
#include "../../newtonian/three_dimensional/default_cell_updater.hpp"
#include "../Voronoi/TetGenTessellation.hpp"
#include "../../newtonian/common/hllc.hpp"
#include "../../newtonian/common/ideal_gas.hpp"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>


using namespace std;

vector<Vector3D> RandSquare(int PointNum, Vector3D const& bll, Vector3D const& fur)
{
	typedef boost::mt19937_64 base_generator_type;
	double ran[3];
	vector<Vector3D> res;
	Vector3D point;
	base_generator_type generator;
	boost::random::uniform_real_distribution<> dist;
	for (int i = 0; i < PointNum; ++i)
	{
		ran[0] = dist(generator);
		ran[1] = dist(generator);
		ran[2] = dist(generator);
		point.x = ran[0] * (fur.x-bll.x) + bll.x;
		point.y = ran[1] * (fur.y - bll.y) + bll.y;
		point.z = ran[2] * (fur.z - bll.z) + bll.z;
		res.push_back(point);
	}
	return res;
}

void main()
{
	int nx=30, ny = 30, nz = 30;
	Vector3D bll(-1, -1, -1), fur(1, 1, 1);
	OuterBoundary3D outer(fur, bll);
	vector<Vector3D> points = RandSquare(nx*ny*nz, bll, fur);
	TetGenTessellation<RigidWallGhostBuster> tess;
	tess.Initialise(points, outer);

	IdealGas eos(5. / 3.);
	Hllc rs;
	Eulerian3D pm;
	FirstOrderHydroFlux flux_calc(rs);
	CourantFriedrichsLewy timestep(0.3);
	DefaultCellUpdater cell_update;

	vector<ComputationalCell> cells(tess.GetPointNo());
	double R_blast = 0.2;
	for (int i = 0; i < tess.GetPointNo(); ++i)
	{
		cells[i].density = 1;
		cells[i].velocity = Vector3D(0, 0, 0);
		if (abs(tess.GetMeshPoint(i)) < R_blast)
			cells[i].pressure = 10000;
		else
			cells[i].pressure = 1;
	}

	// Construct the simulation
	HDSim3D sim(tess,cells, eos, pm, timestep, flux_calc, cell_update);

	//Run the simulation
	for (int i = 0; i < 10; ++i)
	{

		cout << i << endl;
		sim.timeAdvance();
	}

	// Output the data
#ifndef NO_HDF5
	write_snapshot_to_hdf5(sim, "c:/sim_data/final.h5");
#endif

#ifdef _DEBUG
	cout << "Done, press ENTER to finish" << endl;
	string s;
	getline(cin, s);
#endif
}