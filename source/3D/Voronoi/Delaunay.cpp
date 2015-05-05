/*  \file Delaunay.cpp
\brief An abstract class that encapsulates the 3D Delaunay calculations
\Author Itay Zandbank
*/

#include "../Utilities/assert.hpp"
#include "Delaunay.hpp"

Delaunay::Delaunay(const std::vector<VectorRef> &points, const Tetrahedron &bigTetrahedron)
	: _points(points), _bigTetrahedron(bigTetrahedron), _tetrahedra(), _tetrahedraNeighbors(), _vertices()
{

}

void Delaunay::FillVertices()
{
	_vertices.clear();

	for (size_t i = 0; i < _tetrahedra.size(); i++)
		for (size_t j = 0; j < 4; j++)
		{
			VectorRef v = _tetrahedra[i][j];
			VertexMap::iterator existing = _vertices.find(v);
			if (existing == _vertices.end())
			{
				vector<size_t> &newvec = _vertices[v] = vector<size_t>();
				newvec.reserve(10);  // Update the vector that's already in the hash
				newvec.push_back(i);
				// Note - we don't initialize the vector, reserve space and then put it in the hash
				// because putting it in the hash calls the vector copy constructor which doesn't keep
				// the reserved space
			}
			else
				existing->second.push_back(i);
		}
}

const std::vector<size_t> &Delaunay::VertexNeighbors(const VectorRef v) const
{
	VertexMap::const_iterator it = _vertices.find(v);
	BOOST_ASSERT(it != _vertices.end());

	return it->second;
}

void Delaunay::Run()
{
	_tetrahedra.clear();
	_vertices.clear();
	RunDelaunay();
	BOOST_ASSERT(_vertices.size() == 0); // You're not supposed to fill those
	FillVertices();
	FillNeighbors();
}

Tetrahedron Delaunay::CalcBigTetrahedron(const OuterBoundary3D &boundary)
{
	// A big tetrahedron that will contain the bounding box, as well as the 8 adjacent boundary boxes,
	// and with room to spare.

	const Vector3D delta = boundary.FrontUpperRight() - boundary.BackLowerLeft(); // Guaranteed by Boundary to be all positive
	const Vector3D center = boundary.FrontUpperRight() - delta / 2;

	const Vector3D bigDelta = delta * 100;


	// The top of the tetrahedron will be on the Y axis
	vector<VectorRef> tetrahedron;
	tetrahedron.push_back(Vector3D(0, center.y + bigDelta.y, 0));

	// The bottom face is parallel to the x-z plane
	double bottomY = center.y - bigDelta.y;

	// The bottom face is a triangle whose lower edge is parallel to the x axis
	double backZ = center.z - bigDelta.z;
	tetrahedron.push_back(Vector3D(center.x - bigDelta.x, bottomY, backZ));
	tetrahedron.push_back(Vector3D(center.x + bigDelta.x, bottomY, backZ));

	// The last triangle edge is on x=0
	tetrahedron.push_back(Vector3D(center.x, bottomY, center.z + bigDelta.z));

	return Tetrahedron(tetrahedron);
}
