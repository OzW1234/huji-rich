#include <boost/optional.hpp>
#include "../Voronoi/TetGenTessellation.hpp"
#include "../Voronoi/GhostBusters.hpp"

void func()
{
	TetGenTessellation<CloseToBoundaryGhostBuster> voronoi;
	voronoi.Initialise(vector<Vector3D>(), RectangularBoundary3D(Vector3D(1, 1, 1), Vector3D(0, 0, 0)));
}