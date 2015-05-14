/*! \file Delaunay.hpp
   \brief An abstract class that encapsulates the 3D Delaunay calculations
   \author Itay Zandbank
 */

#ifndef DELAUNAY_HPP
#define DELAUNAY_HPP 1

#include <vector>
#include <list>
#include <boost/unordered_map.hpp>
#include "../GeometryCommon/Vector3D.hpp"
#include "../GeometryCommon/Tetrahedron.hpp"
#include "../GeometryCommon/OuterBoundary3D.hpp"

//! \brief An abstract class encapsulating the 3D Delaunay calcluations.
class Delaunay
{
protected:
	std::vector<VectorRef> _points;  //!< The input points of the calculation
	Tetrahedron _bigTetrahedron; //!< The Big Tetrahedron that should surround all the points

	std::vector<Tetrahedron> _tetrahedra;  //!< Resulting Delaunay tetrahedra
	std::vector<std::vector<size_t> > _tetrahedraNeighbors;  //!< Tetrahedra neighbors of each tetrahedron
	
	typedef boost::unordered::unordered_map<VectorRef, std::vector<size_t>, VectorRefHasher> VertexMap;
	VertexMap _vertices; //!< Vector->list of tetrahedra it appears in

	//! \brief Fills the Tetrahedra neighbors
	virtual void FillNeighbors() = 0;
	//! \brief Fills the _vertices vertex-map
	virtual void FillVertices();
	//! \brief Runs the actual Delaunay triangulation
	virtual void RunDelaunay() = 0;

public:
	//! \brief Constucts the Delaunay object
	//! \param points The points to triangulate
	//! \param bigTetrahedron A tetrahedron big enough to include all the points
	Delaunay(const std::vector<VectorRef> &points, const Tetrahedron &bigTetrahedron);

	//! \brief Runs the triangulation
	void Run();

	//! \brief Checks if a point is part of the Big Tetrahedron
	//! \param pt The Point
	//! \return true if the Point is part of the Big Tetrahedron
	bool IsBigTetrahedron(const VectorRef &pt) const
	{
		return std::find(_bigTetrahedron.vertices().begin(), 
			_bigTetrahedron.vertices().end(), pt) != _bigTetrahedron.vertices().end();
	}

	//! \brief A vector of all the tetrahedra
	const std::vector<Tetrahedron> &Tetrahedra() const { return _tetrahedra; }

	//! \brief Returns the neighbors of the tetrahedron
	const std::vector<size_t> &TetrahedraNeighbors(size_t tetrahedron) const { return _tetrahedraNeighbors[tetrahedron]; }

	//! \brief Returns the list of tetrahedra that touch the vertex
	const std::vector<size_t> &VertexNeighbors(const VectorRef v) const;

	//! \brief The number of tetrahedra.
	size_t NumTetrahedra() const { return _tetrahedra.size(); }

	//! \brief Accesses a tetrahedron
	const Tetrahedron& operator[](size_t index) const { return _tetrahedra[index]; }

	//! \brief The Big Tetrahedron
	const Tetrahedron& BigTetrahedron() const { return _bigTetrahedron; }

	//! \brief The original input points.
	const std::vector<VectorRef> &InputPoints() const { return _points; }

	//! \brief Returns a Big Tetrahedron covering the entire boundary and its 26 adjacent subcubes - with room to spare
	static Tetrahedron CalcBigTetrahedron(const OuterBoundary3D &boundary);
};

#endif // DELAUNAY_HPP