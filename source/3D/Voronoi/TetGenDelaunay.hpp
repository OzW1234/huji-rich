/*!
 \file TetGenDelaunay.hpp
 \brief a TetGen based implementation of the Delaunay abstract class
 \author Itay Zandbank
 */

#ifndef TETGENDELAUNAY_HPP
#define TETGENDELAUNAY_HPP

#include "Delaunay.hpp"
#include "../GeometryCommon/Face.hpp"

//! \brief The actual private implementation of the Tetgen interface
class TetGenImpl;

class TetGenDelaunay : public Delaunay
{
public:
	//! \brief Constructor
	//! \param points The Mesh Points
	//! \param bigTetrahedron The Big Tetrahedron
	//! \param runVoronoi When true, the Voronoi Tessellation will be generated as well
	TetGenDelaunay(const std::vector<VectorRef> &points, const Tetrahedron &bigTetrahedron, bool runVoronoi=true);
	
	//! \brief Returns all the constructed cell faces.
	std::vector<Face> GetVoronoiFaces() const;

	//! \brief Returns the indices of the faces of a cell
	//! \param cellNum The cell number
	//! \return The indices of the cell's faces
	std::vector<size_t> GetVoronoiCellFaces(size_t cellNum) const;

protected:
	void RunDelaunay();
	virtual void FillNeighbors();

private:
	friend class TetGenImpl;   // This is the PIMPL pattern, although the PIMPL is generated once per call to Run,
	// so we don't need to hold a pointer to it.

	// Voronoi output
	std::vector<std::vector<int> > _voronoiCellFaces;  // Use ints for all the vectors, because that's what tetgen uses internally.
	std::vector<std::vector<int> > _voronoiFaceEdges;  // Sometimes -1 is used as a marker
	std::vector<std::pair<int, int> > _voronoiFaceNeighbors; // Neighbors of each face
	std::vector<std::pair<int, int> > _voronoiEdges;
	std::vector<VectorRef> _voronoiVertices;

	Face GetVoronoiFace(size_t faceNum) const;
	bool _runVoronoi;
};

#endif // TETGEN_DELAUNAY_HPP