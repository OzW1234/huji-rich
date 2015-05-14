#include "Face.hpp"
#include "../../misc/universal_error.hpp"
#include <sstream>
#include <cmath>

using namespace std;

Face::Face(const std::vector<VectorRef> &vert, boost::optional<size_t> neighbor1, boost::optional<size_t> neighbor2) :
	_neighbor1(neighbor1), _neighbor2(neighbor2), _area(), _centroid(), vertices(vert)
{
}

const Face Face::Empty = Face(vector<VectorRef>());  // An empty face with no vertices

Face::Face(const Face &other):
  _neighbor1(other._neighbor1),
  _neighbor2(other._neighbor2),
  _area(other._area),
  _centroid(other._centroid),
  vertices(other.vertices)
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
	for (size_t i = 0; i < vertices.size(); i++)
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

bool operator==(const Face &face1, const Face &face2)
{
	if (face1.vertices.size()!=face2.vertices.size())
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