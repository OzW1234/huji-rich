/* \file Delaunay.hpp
   \brief An abstract class that encapsulates the 3D Delaunay calculations
   \Author Itay Zandbank
 */

#ifndef DELAUNAY_HPP
#define DELAUNAY_HPP 1

#include <vector>
#include <list>
#include <unordered_map>
#include "../GeometryCommon/Vector3D.hpp"
#include "../GeometryCommon/Tetrahedron.hpp"
#include "../GeometryCommon/OuterBoundary3D.hpp"

class Delaunay
{
protected:
protected:
	std::vector<VectorRef> _points;
	Tetrahedron _bigTetrahedron;

	std::vector<Tetrahedron> _tetrahedra;
	std::vector<std::vector<size_t>> _tetrahedraNeighbors;  // Tetrahedra neighbors of each tetrahedron
	
	typedef std::unordered_map<VectorRef, std::vector<size_t>> VertexMap;
	VertexMap _vertices; // Vector->list of tetrahedra it appears in

	virtual void FillNeighbors() = 0;
	virtual void FillVertices();
	virtual void RunDelaunay() = 0;

public:
	Delaunay(const std::vector<VectorRef> &points, const Tetrahedron &bigTetrahedron);
	void Run();

	bool IsBigTetrahedron(const VectorRef &pt) const
	{
		return std::find(_bigTetrahedron.vertices().begin(), 
			_bigTetrahedron.vertices().end(), pt) != _bigTetrahedron.vertices().end();
	}

	//\brief A vector of all the tetrahedra
	const std::vector<Tetrahedron> &Tetrahedra() const { return _tetrahedra; }

	//\brief Returns the neighbors of the tetrahedron
	const std::vector<size_t> &TetrahedraNeighbors(size_t tetrahedron) const { return _tetrahedraNeighbors[tetrahedron]; }

	//\brief Returns the list of tetrahedra that touch the vertex
	const std::vector<size_t> &VertexNeighbors(const VectorRef v) const;

	size_t NumTetrahedra() const { return _tetrahedra.size(); }
	const Tetrahedron& operator[](size_t index) const { return _tetrahedra[index]; }

	const Tetrahedron& BigTetrahedron() const { return _bigTetrahedron; }
	const std::vector<VectorRef> &InputPoints() const { return _points; }

	//\brief Returns a Big Tetrahedron covering the entire boundary and its 26 adjacent subcubes - with room to spare
	static Tetrahedron CalcBigTetrahedron(const OuterBoundary3D &boundary);
};

#endif // DELAUNAY_HPP