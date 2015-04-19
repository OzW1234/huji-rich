//\file TetGenTessellation.hpp
//\brief Uses TetGen and a ghost buster for the entire Voronoi tessallation
//\author Itay Zandbank

#ifndef TETGENTESSELLATION_HPP
#define TETGENTESSELLATION_HPP

#include "TessellationBase.hpp"
#include "GhostBusters.hpp"
#include "TetGenDelaunay.hpp"
#include "../GeometryCommon/CellCalculations.hpp"
#include <boost/shared_ptr.hpp>
#include <unordered_set>

template <typename GhostBusterType>
class TetGenTessellation : public TessellationBase
{
private:
	// Make sure GhostBusterType is derived from GhostBuster. If you see an error on this line, you've instantiated the template
	// with a wrong template argument
	BOOST_STATIC_ASSERT(boost::is_base_of<typename GhostBuster, typename GhostBusterType>::value);

public:
	virtual void Update(vector<Vector3D> const& points);

	virtual Tessellation3D* clone(void) const;
	virtual const vector<GhostPointInfo> &GetDuplicatedPoints() const ;
	virtual size_t GetTotalPointNumber() const;
	virtual bool IsGhostPoint(size_t index) const;

	std::vector<VectorRef> AllPoints;
	const static double EDGE_RATIO;

	virtual Vector3D GetMeshPoint(size_t index) const;
	virtual const vector<Vector3D> &GetMeshPoints() const;

private:
	void ConvertToVoronoi(const TetGenDelaunay &del);

	// TetGenDelaunay creates Voronoi cells for all the ghost points, too. We just need the cells of the original
	// points, which are the first _meshPoints.size() cells, and the faces that  are part of these cells.
	std::vector<boost::shared_ptr<Face>> _allFaces;  // All relevant faces
	std::vector<double> _cellVolumes;		    // Volumes of the TetGenDelaunay calculated cells

	std::vector<Vector3D> _allPoints;			// Includes the mesh points followed by the ghost points
	std::vector<GhostPointInfo> _ghostPointInfo;   // Includes all the mesh points and ghost points

	void ExtractTetGenFaces(const TetGenDelaunay &del);
	void CalculateTetGenVolumes(const TetGenDelaunay &del);
	void OptimizeFace(size_t faceNum); // Optimize the face in place

	//\brief Constructs all the cell objects
	void ConstructCells(const TetGenDelaunay &del);
};

template<typename GhostBusterType>
const double TetGenTessellation<GhostBusterType>::EDGE_RATIO = 1e-5;

template<typename GhostBusterType>
Vector3D TetGenTessellation<GhostBusterType>::GetMeshPoint(size_t index) const
{
	return _allPoints[index];
}

template<typename GhostBusterType>
const std::vector<Vector3D> &TetGenTessellation<GhostBusterType>::GetMeshPoints() const
{
	return _allPoints;
}

template<typename GhostBusterType>
Tessellation3D *TetGenTessellation<GhostBusterType>::clone() const
{
	return new TetGenTessellation<GhostBusterType>(*this);
}

template<typename GhostBusterType>
const vector<Tessellation3D::GhostPointInfo>& TetGenTessellation<GhostBusterType>::GetDuplicatedPoints() const
{
	return _ghostPointInfo;
}

template<typename GhostBusterType>
size_t TetGenTessellation<GhostBusterType>::GetTotalPointNumber() const
{
	return _allPoints.size();
}

template<typename GhostBusterType>
bool TetGenTessellation<GhostBusterType>::IsGhostPoint(size_t index) const
{
	return index < 0 || index >= GetPointNo();
}

template<typename GhostBusterType>
void TetGenTessellation<GhostBusterType>::Update(const vector<Vector3D> &points)
{
	CheckBoundaryConformance(points);
	_meshPoints = points;
	ClearData();

	Tetrahedron big = Delaunay::CalcBigTetrahedron(*_boundary);
	vector<VectorRef> pointRefs(points.begin(), points.end());

	// First phase
	TetGenDelaunay del1(pointRefs, big, false);
	del1.Run();

	// Find the ghost points
	GhostBusterType ghostBuster;
	GhostBuster::GhostMap ghosts = ghostBuster(del1, *_boundary);

	// Now the second phase, with the ghost points.
	// We fill the _allPoints and _ghostPointInfo structures
	_allPoints.clear();
	_allPoints.reserve(pointRefs.size() + ghosts.size());
	_ghostPointInfo.clear();
	_ghostPointInfo.reserve(pointRefs.size() + ghosts.size());

	// Start with the mesh points
	Subcube meshPointSubcube("   ");
	for (vector<VectorRef>::const_iterator itPoint = pointRefs.begin(); itPoint != pointRefs.end(); itPoint++)
	{
		_allPoints.push_back(**itPoint);
		GhostPointInfo gpi(_allPoints.size(), _allPoints.size(), meshPointSubcube, **itPoint);
		_ghostPointInfo.push_back(gpi);
	}

	// Now the ghost points
	for (GhostBuster::GhostMap::iterator itMap = ghosts.begin(); itMap != ghosts.end(); itMap++)
	{
		VectorRef originalPoint = itMap->first;
		for (GhostBuster::GhostMap::mapped_type::iterator itSet = itMap->second.begin(); itSet != itMap->second.end(); itSet++)
		{
			Subcube subcube = itSet->first;
			VectorRef ghostRef = itSet->second;
			_allPoints.push_back(*ghostRef);
			
			GhostPointInfo gpi(_allPoints.size(), 
				*GetPointIndex(originalPoint),
				subcube,
				*ghostRef);
			_ghostPointInfo.push_back(gpi);
		}
	}

	AllPoints = VectorRef::vector(_allPoints);
	TetGenDelaunay del2(AllPoints, big, true);
	del2.Run();

	ConvertToVoronoi(del2);
}

template<typename GhostBusterType>
void TetGenTessellation<GhostBusterType>::ConvertToVoronoi(const TetGenDelaunay &del)
{
	ExtractTetGenFaces(del);
	CalculateTetGenVolumes(del);

	for (size_t faceNum = 0; faceNum < _allFaces.size(); faceNum++)
		OptimizeFace(faceNum);

	ConstructCells(del);
}

template<typename GhostBusterType>
void TetGenTessellation<GhostBusterType>::ExtractTetGenFaces(const TetGenDelaunay &del)
{
	const std::vector<Face> &_tetgenFaces = del.GetVoronoiFaces();

	_allFaces.resize(_tetgenFaces.size());
	for (size_t cellNum = 0; cellNum < _meshPoints.size(); cellNum++)
	{
		const std::vector<size_t> &faceIndices = del.GetVoronoiCellFaces(cellNum);
		for (std::vector<size_t>::const_iterator it = faceIndices.begin(); it != faceIndices.end(); it++)
			if (!_allFaces[*it])
				_allFaces[*it].reset(new Face(_tetgenFaces[*it]));
	}
}

template<typename GhostBusterType>
void TetGenTessellation<GhostBusterType>::CalculateTetGenVolumes(const TetGenDelaunay &del)
{
	_cellVolumes.clear();
	_allCMs.clear();

	for (size_t cellNum = 0; cellNum < _meshPoints.size(); cellNum++)
	{
		std::vector<const Face *> facePointers;
		const std::vector<size_t> &faceIndices = del.GetVoronoiCellFaces(cellNum);
		for (std::vector<size_t>::const_iterator itFace = faceIndices.begin(); itFace != faceIndices.end(); itFace++)
		{
			const Face &face = *_allFaces[*itFace];
			facePointers.push_back(&face);
		}

		double volume;
		Vector3D CoM;
		CalculateCellDimensions(facePointers, volume, CoM);
		_cellVolumes.push_back(volume);
		_allCMs.push_back(CoM);
	}
}

//\brief Optimize a face
//\remark Removes edges that are too short, and then remove faces that are degenerate
//TODO: Remove faces whose area is too small
// We compare each edge to the geometric average of the radius of the two adjacent cells
template<typename GhostBusterType>
void TetGenTessellation<GhostBusterType>::OptimizeFace(size_t faceNum)
{
	if (!_allFaces[faceNum])
		return;

	const Face &face = *_allFaces[faceNum];
	BOOST_ASSERT(face.NumNeighbors() == 2); // All our faces should have two neighbors, although one of them may be a ghost cell

	size_t neighbor1 = *face.Neighbor1();
	size_t neighbor2 = *face.Neighbor2();
	if (neighbor1 > neighbor2)
		std::swap(neighbor1, neighbor2);

	BOOST_ASSERT(neighbor1 < _meshPoints.size()); // At least one neighbor shouldn't be for a ghost
	double volume1 = _cellVolumes[neighbor1];
	double volume2 = neighbor2 < _meshPoints.size() ? _cellVolumes[neighbor2] : volume1;
	double combined = pow(volume1 * volume2, 1.0 / 6.0);  // Cubic Root of geometric average of volumes

	double threshold = combined * EDGE_RATIO;
	double threshold2 = threshold * threshold; // We combine the threshold to the distance, this saves the sqrt operations
	std::vector<VectorRef> vertices; // Vertices we keep in our face
	vertices.reserve(face.vertices.size());

	VectorRef prevVertex = face.vertices.back();
	for (std::vector<VectorRef>::const_iterator it = face.vertices.begin(); it != face.vertices.end(); it++)
	{
		double distance2 = abs2(**it - *prevVertex);
		if (distance2 > threshold2)  // Keep this vertex
		{
			vertices.push_back(*it);
			prevVertex = *it;
		}
	}

	if (vertices.size() == face.vertices.size())  // No change in the face
		return;
	
	if (vertices.size() < 3)  // Degenerate face
		_allFaces[faceNum].reset();
	else
	{
		_allFaces[faceNum].reset(new Face(vertices, face.Neighbor1(), face.Neighbor2()));
	}
}

template<typename GhostBusterType>
void TetGenTessellation<GhostBusterType>::ConstructCells(const TetGenDelaunay &del)
{
	_faces.Reserve(_meshPoints.size() * 15);  // An average of 15 faces per cell

	for (size_t cellNum = 0; cellNum < _meshPoints.size(); cellNum++)
	{
		std::vector<size_t> ourFaceIndices;

		const std::vector<size_t> &tetGenFaceNums = del.GetVoronoiCellFaces(cellNum);
		for (std::vector<size_t>::const_iterator it = tetGenFaceNums.begin(); it != tetGenFaceNums.end(); it++)
		{
			if (!_allFaces[*it])  // Face was removed
				continue;

			const Face &tetGenFace = *_allFaces[*it];
			BOOST_ASSERT(tetGenFace.NumNeighbors() == 2);  // Sanity check, this has been checked previously
			size_t ourFaceIndex = _faces.StoreFace(tetGenFace.vertices, *tetGenFace.Neighbor1(), *tetGenFace.Neighbor2());
			ourFaceIndices.push_back(ourFaceIndex);
		}

		// Build the cells using the old pre-optimized face calculations - the changes are very very small
		_cells[cellNum] = new Cell(ourFaceIndices, _cellVolumes[cellNum], _meshPoints[cellNum], _allCMs[cellNum]);
	}
}

#endif