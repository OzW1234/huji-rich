#include "TessellationBase.hpp"
#include <set>
#include <unordered_set>
#include <sstream>

static const double BOUNDARY_REGION = 1e-8;

/*
* The FaceStore
*/

size_t TessellationBase::FaceStore::StoreFace(const vector<VectorRef> &vertices, size_t neighbor1, size_t neighbor2)
{
	Face *p = new Face(vertices, neighbor1, neighbor2);

	size_t index = _faces.size();
	_faces.push_back(p);
	return index;
}

TessellationBase::FaceStore::~FaceStore()
{
	Clear();
}

void TessellationBase::FaceStore::Clear()
{
	for (vector<Face *>::iterator it = _faces.begin(); it != _faces.end(); it++)
		delete *it;
	_faces.clear();
}

TessellationBase::Cell::Cell(const std::vector<size_t> &faces, double volume, VectorRef center, VectorRef centerOfMass) :
	_faces(faces), _volume(volume),_center(center), _centerOfMass(centerOfMass)
{
	// width is the radius of the sphere with the same volume as the cell
	// volume = (4/3) * radius ^ 3
	_width = pow(3.0 / 4.0 * volume, 1.0 / 3.0);
}

void TessellationBase::ClearCells()
{
	for (vector<Cell *>::iterator it = _cells.begin(); it != _cells.end(); it++)
	{
		if (*it)
			delete *it;
	}
	_cells.clear();
}

void TessellationBase::ClearData()
{
	_faces.Clear();
	ClearCells();
	_cells.resize(_meshPoints.size());
	_allCMs.clear();
	_allCMs.resize(_meshPoints.size());
	FillPointIndices();
}

TessellationBase::~TessellationBase()
{
	ClearCells();
}

void TessellationBase::Initialise(vector<Vector3D> const& points, const OuterBoundary3D &bc)
{
	_boundary = &bc;
	Update(points);
}

void TessellationBase::CheckBoundaryConformance(const vector<Vector3D> &points) const
{
	Vector3D thresholds = (_boundary->FrontUpperRight() - _boundary->BackLowerLeft()) * 1e-9;

	Subcube left("-  "), right("+   "), over(" + "), below(" - "), infront("  +"), behind("  -");

	for (vector<Vector3D>::const_iterator itPt = points.begin(); itPt != points.end(); itPt++)
	{
		if (_boundary->distance(*itPt, left) < thresholds.x || _boundary->distance(*itPt, right) < thresholds.x ||
			_boundary->distance(*itPt, over) < thresholds.y || _boundary->distance(*itPt, below) < thresholds.y ||
			_boundary->distance(*itPt, infront) < thresholds.z || _boundary->distance(*itPt, behind) < thresholds.z)
		{
			stringstream ss;

			ss << "Point " << *itPt << " is too close to boundary";
			throw invalid_argument(ss.str());
		}
	}
}


size_t TessellationBase::GetPointNo(void) const
{
	return _meshPoints.size();
}

Vector3D TessellationBase::GetMeshPoint(size_t index) const
{
	return _meshPoints[index];
}

Vector3D const& TessellationBase::GetCellCM(size_t index) const
{
	return *_cells[index]->GetCenterOfMass();
}

/*! \brief Returns the total number of faces
\return Total number of faces
*/
size_t TessellationBase::GetTotalFacesNumber(void) const
{
	return _faces.NumFaces();
}

const Face& TessellationBase::GetFace(size_t index) const
{
	return _faces.GetFace(index);
}

double TessellationBase::GetWidth(size_t index) const
{
	return _cells[index]->GetWidth();
}

double TessellationBase::GetVolume(size_t index) const
{
	return _cells[index]->GetVolume();
}

vector<size_t>const& TessellationBase::GetCellFaces(size_t index) const
{
	return _cells[index]->GetFaces();
}

vector<Vector3D>& TessellationBase::GetMeshPoints(void)
{
	return _meshPoints;
}

vector<size_t> TessellationBase::GetNeighbors(size_t index) const
{
	if (index >= GetPointNo())
		return vector<size_t>();  // No neighbors for ghost cells
	const Cell& cell = *_cells[index];
	vector<size_t> neighbors;

	const vector<size_t> &faceIndices = _cells[index]->GetFaces();
	for (size_t i = 0; i < faceIndices.size(); i++)
	{
		const Face &face = GetFace(i);
		if (face.Neighbor1() == index)
			neighbors.push_back(*face.Neighbor2());
		else if (face.Neighbor2() == index)
			neighbors.push_back(*face.Neighbor1());
		else
			BOOST_ASSERT(false); // One of the neighbors must be us!
	}

	return neighbors;
}


vector<Vector3D>& TessellationBase::GetAllCM()
{
	return _allCMs;
}

void TessellationBase::GetNeighborNeighbors(vector<size_t> &result, size_t point) const
{
	unordered_set<size_t> allNeighbors;
	const vector<size_t> &neighbors = GetNeighbors(point);
	for (vector<size_t>::const_iterator it = neighbors.begin(); it != neighbors.end(); it++)
	{
		allNeighbors.insert(*it);
		if (IsGhostPoint(*it))
		{
			vector<size_t> neighbors2 = GetNeighbors(*it);
			allNeighbors.insert(neighbors2.begin(), neighbors2.end());
		}
	}

	// See here: http://stackoverflow.com/a/5034274/871910
	result.clear();
	copy(allNeighbors.begin(), allNeighbors.end(), back_inserter(result));
}


Vector3D TessellationBase::Normal(size_t faceIndex) const
{
	const Face &face = GetFace(faceIndex);
	Vector3D center1 = GetMeshPoint(*face.Neighbor1());
	Vector3D center2 = GetMeshPoint(*face.Neighbor2());

	return center2 - center1;
}

boost::optional<size_t> TessellationBase::FindFaceWithNeighbors(size_t n0, size_t n1) const
{
	BOOST_ASSERT(n0 != n1);
	BOOST_ASSERT(!IsGhostPoint(n0) || !IsGhostPoint(n1));

	if (n0 > n1)
		swap(n0, n1);
	BOOST_ASSERT(!IsGhostPoint(n0));

	const std::vector<size_t>& cellFaces = _cells[n0]->GetFaces();  // Guaranteed not to be a ghost point
	for (std::vector<size_t>::const_iterator itFace = cellFaces.begin(); itFace != cellFaces.end(); itFace++)
	{
		const Face &face = _faces.GetFace(*itFace);
		if (face.OtherNeighbor(n0) == n1)
			return *itFace;
	}

	return boost::none;
}

/*!
\brief Calculates the velocity of a face
\param p0 The index of the first neighbor
\param p1 The index of the second neighbor
\param v0 The velocity of the first neighbor
\param v1 The velocity of the second neighbor
\return The velocity of the face
*/
Vector3D TessellationBase::CalcFaceVelocity(size_t p0, size_t p1, Vector3D const& v0,
	Vector3D const& v1) const
{

	boost::optional<size_t> faceIndex = FindFaceWithNeighbors(p0, p1); //  _faces.FindFace(p0, p1);
	if (!faceIndex.is_initialized())
		throw invalid_argument("Can't find face");

	const Face &face = _faces.GetFace(*faceIndex);

	Vector3D r0 = GetMeshPoint(p0);
	Vector3D r1 = GetMeshPoint(p1);
	Vector3D r_diff = r1 - r0;
	double abs_r_diff = abs(r_diff);

	Vector3D f = face.GetCentroid();

	Vector3D delta_w = ScalarProd((v0 - v1), (f - r_diff / 2)) * r_diff / abs_r_diff;
	Vector3D w = (v0 + v1) / 2 + delta_w;
	return w;
}

//\brief Check if a cell touches the boundary
//\remarks The cell touches a boundary iff it has a boundary face
bool TessellationBase::NearBoundary(size_t index) const
{
	const vector<size_t> faces = GetCellFaces(index);
	for (vector<size_t>::const_iterator it = faces.begin(); it != faces.end(); it++)
		if (BoundaryFace(*it))
			return true;
	return false;
}

//\brief Checks if a face is a Boundary face
//\remarks a Face is a Boundary face iff it only has one neighbor
bool TessellationBase::BoundaryFace(size_t index) const
{
	const Face &face = GetFace(index);
	return IsGhostPoint(*face.Neighbor1()) || IsGhostPoint(*face.Neighbor2());
}

//\brief Fills the pointIndices map
void TessellationBase::FillPointIndices()
{
	_pointIndices.clear();
	for (size_t i = 0; i < _meshPoints.size(); i++)
		_pointIndices[_meshPoints[i]] = i;
}

boost::optional<size_t> TessellationBase::GetPointIndex(const VectorRef pt) const
{
	unordered_map<VectorRef, size_t, VectorRefHasher>::const_iterator it = _pointIndices.find(pt);
	if (it != _pointIndices.end())
		return it->second;

	return boost::none;
}

void TessellationBase::GetTetrahedronIndices(const Tetrahedron &t, boost::optional<size_t> *cells) const
{
	for (int i = 0; i < 4; i++)
		cells[i] = GetPointIndex(t[i]);
}

