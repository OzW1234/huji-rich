#include "Face.hpp"
#include "../../misc/universal_error.hpp"
#include <sstream>
#include <cmath>

using namespace std;

Face::Face(const vector<VectorRef> &vert, boost::optional<size_t> neighbor1, boost::optional<size_t> neighbor2) :
	vertices(vert), _neighbor1(neighbor1), _neighbor2(neighbor2)
{

}

const Face Face::Empty = Face(vector<VectorRef>());  // An empty face with no vertices

Face::Face(const Face &other):
  vertices(other.vertices),
  _neighbor1(other._neighbor1) ,
  _neighbor2(other._neighbor2)
{
}

void Face::CalculateArea() const
{
	double res = 0;
	for (size_t i = 0; i<vertices.size() - 2; ++i)
		res += 0.5*abs(CrossProduct(*vertices[i + 1] - *vertices[0], *vertices[i + 2] - *vertices[0]));
	_area = res;
}

double Face::GetArea() const
{
	if (!_area.is_initialized())
		CalculateArea();
	return *_area;
}

void Face::CalculateCentroid() const
{
	// The face is convex, we find the middle and break it into triangles
	Vector3D middle;
	for (vector<VectorRef>::const_iterator itVertex = vertices.begin(); itVertex != vertices.end(); itVertex++)
		middle = middle + **itVertex;
	middle = middle / (double)vertices.size();

	// Now split into triangles - (middle, V0, V1), (middle, V1, V2), (middle, V2, V3) ... (middle, Vn, V0)
	double totalArea = 0.0;
	Vector3D centroid;
	for (int i = 0; i < vertices.size(); i++)
	{
		// The triangle
		Vector3D a = middle;
		Vector3D b = *vertices[i];
		Vector3D c = *vertices[(i + 1) % vertices.size()];

		// Its centroid and area
		Vector3D triangleCentroid = (a + b + c) / 3;  // See here: http://mathforum.org/library/drmath/view/54899.html
		Vector3D ab = a - b;
		Vector3D bc = b - c;
		double area = 0.5 * abs(CrossProduct(ab, bc)); // See here: http://geomalgorithms.com/a01-_area.html

		// The weighted average
		totalArea += area;
		centroid = centroid + area * triangleCentroid;
	}

	_centroid = centroid / totalArea;
}

Vector3D Face::GetCentroid() const
{
	if (!_centroid.is_initialized())
		CalculateCentroid();
	return *_centroid;
}

bool Face::IdenticalTo(const vector<VectorRef> &otherVertices) const
{
	size_t size = (int)otherVertices.size();
	BOOST_ASSERT(size >= 3);

	if (vertices.size() != size)
		return false;

	// First find the index of each of our vertices in otherVertices
	vector<int> indices(size);
	for (size_t i = 0; i < size; i++)
	{
		vector<VectorRef>::const_iterator it = std::find(otherVertices.begin(), otherVertices.end(), vertices[i]);
		if (it == otherVertices.end())  // Can't find vertex in the other face
			return false;
		indices[i] = (int)std::distance(otherVertices.begin(), it);
	}

	// The indices should be either x, x+1 , x+2, ... x+size-1 (all modulu size), or
	// x, x-1, x-2, ...., x-size+1 (all modulo size) if the faces are in opposite directions.
	// So the differences between the indices should be
	// 1, 1, 1, 1, 1, -size, 1, 1, 1, 1 or -1, -1, -1, -1, size, -1, -1, -1, -1
	// with size being somewhere in there (may be in the middle, may be on one of the ends)
	bool sameDirection;
	int diff = indices[1] - indices[0];
	if (diff == 1 || diff == -size+1)
		sameDirection = true;
	else if (diff == -1 || diff == size-1)
		sameDirection = false;
	else
		return false;   // The faces are *not* identical

	bool sizeFound = false;
	for (int i = 0; i < size; i++)
	{
		int diff = indices[(i + 1) % size] - indices[i];
		if (!sameDirection)
			diff *= -1;
		if (diff != 1 && diff != -size + 1)
			return false; 
		if (diff == -size + 1)
		{
			if (sizeFound)
				return false;
			sizeFound = true;
		}
	}

	return sizeFound;
}

Vector3D calc_centroid(const Face& face)
{
	Vector3D res;
	for(size_t i=0;i<face.vertices.size()-2;++i)
	{
		double area=0.5*abs(CrossProduct(*face.vertices[i+1]-*face.vertices[0],
			*face.vertices[i+2]-*face.vertices[0]));
		res.x+=area*(face.vertices[0]->x+face.vertices[i+2]->x+face.vertices[i+1]->x)/3.;
		res.y+=area*(face.vertices[0]->y+face.vertices[i+2]->y+face.vertices[i+1]->y)/3.;
		res.z+=area*(face.vertices[0]->z+face.vertices[i+2]->z+face.vertices[i+1]->z)/3.;
	}
	return res;
}

static const double PI = acos(-1);  // No PI definition in the C++ standard!!!
static const double EPSILON = 1e-12;

double Face::FullAngle(const Vector3D &v1, const Vector3D &v2)
{
	double angle = CalcAngle(v1, v2);
	Vector3D cross = CrossProduct(v1, v2);

	if (cross.x < 0 ||
		cross.x == 0 && cross.y < 0 ||
		cross.x == 0 && cross.y == 0 && cross.z < 0)
	{
		// This means the angle is between Pi and 2*Pi - adjust it
		angle = 2 * PI - angle;
	}

	return angle;
}

typedef std::pair<double, VectorRef> AngledVertex;

static bool CompareAngledVertices(const AngledVertex &a1, const AngledVertex &a2)
{
	double diff = a1.first - a2.first;
	return diff < -EPSILON;
}

Face Face::ReorderVertices()
{
	Vector3D center;
	std::vector<VectorRef> reordered;

	// We need to sort the vectors and angles together (no standard C++ sort returns the sorting permutation)
	std::vector<std::pair<double, VectorRef>> angledVertices(vertices.size());

	for (size_t i = 0; i < vertices.size(); i++)
		center += *vertices[i];
	center = center / (int)vertices.size();

	Vector3D line0FromCenter = center - *vertices[0];
	for (size_t i = 0; i < vertices.size(); i++)
	{
		Vector3D lineFromCenter = center - *vertices[i];
		angledVertices[i].first = FullAngle(line0FromCenter, lineFromCenter);
		angledVertices[i].second = vertices[i];
	}

	std::sort(angledVertices.begin(), angledVertices.end(), CompareAngledVertices);
	
	for (size_t i = 0; i < vertices.size(); i++)  // Copy the results
		reordered.push_back(angledVertices[i].second);

	return Face(reordered, _neighbor1, _neighbor2);
}

bool operator==(const Face &face1, const Face &face2)
{
	size_t sizeFace1 = face1.vertices.size();
	size_t sizeFace2 = face2.vertices.size();
	if (sizeFace1!=sizeFace2)
		return false;

	//	if (face1.vertices.size() != face2.vertices.size())
	std::hash<Face> hasher;
	size_t hash1 = hasher(face1);
	size_t hash2 = hasher(face2);
	//if (hasher(face1) != hasher(face2))
	if (hash1!=hash2)
		return false;

	// This is a naive O(N^2) implementation, which beats a clever O(N) with hash tables hands down,
	// because N is very small, and building set<>s or unordered_set<>s is expensive
	for (vector<VectorRef>::const_iterator it1 = face1.vertices.begin(); it1 != face1.vertices.end(); it1++)
	{
		if (find(face2.vertices.begin(), face2.vertices.end(), *it1) == face2.vertices.end())
			return false;
	}
	return true;
}