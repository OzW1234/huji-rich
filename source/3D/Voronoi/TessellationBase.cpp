#include "TessellationBase.hpp"
#include <set>
#include <sstream>

static const double BOUNDARY_REGION = 1e-8;

/*
* The FaceStore
*/

bool TessellationBase::FaceStore::FindFace(const Face &face, size_t &index) const
{
	for (index = 0; index < _faces.size(); index++)
		if (_faces[index]==face)
			return true;

	return false;
}

size_t TessellationBase::FaceStore::StoreFace(const vector<VectorRef> &vertices)
{
	Face face(vertices);
	size_t index;
	bool exists = FindFace(face, index);
	if (exists)
		return index;

	index = _faces.size();
	_faces.push_back(face);
	return index;
}

void TessellationBase::FaceStore::Clear()
{
	_faces.clear();
}

boost::optional<size_t> TessellationBase::FaceStore::FindFace(size_t Neighbor1, size_t Neighbor2) const
{
	// TODO: Make this more efficient, this is bound to cause performance problems
	for (vector<Face>::const_iterator it = _faces.begin(); it != _faces.end(); it++)
	{
		if (it->Neighbor1() == Neighbor1 && it->Neighbor2() == Neighbor2 ||
			it->Neighbor1() == Neighbor2 && it->Neighbor2() == Neighbor1)
		{
			return it - _faces.begin();
		}
	}

	return boost::none;
}

TessellationBase::Cell::Cell(std::vector<size_t> faces, double volume, VectorRef center, VectorRef centerOfMass) :
	_faces(faces), _volume(volume),_center(center), _centerOfMass(centerOfMass)
{
	// width is the radius of the sphere with the same volume as the cell
	// volume = (4/3) * radius ^ 3
	_width = pow(3 / 4 * volume, 1 / 3);
}

void TessellationBase::ClearData()
{
	_faces.Clear();
	_cells.clear();
	_cells.resize(_meshPoints.size());
	_allCMs.clear();
	_allCMs.resize(_meshPoints.size());
	FillPointIndices();
}

void TessellationBase::Initialise(vector<Vector3D> const& points, const OuterBoundary3D &bc)
{
	_boundary = &bc;
	Update(points);
}

void TessellationBase::CheckBoundaryConformance(const vector<Vector3D> &points) const
{
	const char *offsets[] = { "-  ", "+  ", " - ", " + ", "  -", "  +" };
	static vector<Subcube> subcubes;

	if (subcubes.size() == 0)
		for (int i = 0; i < 6; i++)
			subcubes.push_back(Subcube(offsets[i]));

	for (vector<Vector3D>::const_iterator itPt = points.begin(); itPt != points.end(); itPt++)
		for (vector<Subcube>::const_iterator itSubcube = subcubes.begin(); itSubcube != subcubes.end(); itSubcube++)
		{
			double dist = _boundary->distance(*itPt, *itSubcube);
			if (dist < BOUNDARY_REGION)
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
	return *_cells[index].GetCenterOfMass();
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
	return _cells[index].GetWidth();
}

double TessellationBase::GetVolume(size_t index) const
{
	return _cells[index].GetVolume();
}

vector<size_t>const& TessellationBase::GetCellFaces(size_t index) const
{
	return _cells[index].GetFaces();
}

vector<Vector3D>& TessellationBase::GetMeshPoints(void)
{
	return _meshPoints;
}

vector<size_t> TessellationBase::GetNeighbors(size_t index) const
{
	if (index >= GetPointNo())
		return vector<size_t>();  // No neighbors for ghost cells
	const Cell& cell = _cells[index];
	vector<size_t> neighbors;

	auto faceIndices = _cells[index].GetFaces();
	for (size_t i = 0; i < faceIndices.size(); i++)
	{
		auto face = GetFace(i);
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
	set<size_t> allNeighbors;
	vector<size_t> neighbors = GetNeighbors(point);
	for (auto it = neighbors.begin(); it != neighbors.end(); it++)
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
	Face face = GetFace(faceIndex);
	Vector3D center1 = GetMeshPoint(*face.Neighbor1());
	Vector3D center2 = GetMeshPoint(*face.Neighbor2());

	return center2 - center1;
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
	boost::optional<size_t> faceIndex = _faces.FindFace(p0, p1);
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
	Face face = GetFace(index);
	return face.NumNeighbors() == 1;
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
	unordered_map<VectorRef, size_t>::const_iterator it = _pointIndices.find(pt);
	if (it != _pointIndices.end())
		return it->second;

	return boost::none;
}

void TessellationBase::GetTetrahedronIndices(const Tetrahedron &t, boost::optional<size_t> *cells) const
{
	for (int i = 0; i < 4; i++)
		cells[i] = GetPointIndex(t[i]);
}

