/* \file Tetrahedron.hpp
 \brief A simple encapsulation of a tetrahedron 
 \author Itay Zandbank
*/

#ifndef TETRAHEDRON_HPP
#define TETRAHEDRON_HPP

#include <vector>
#include "Vector3D.hpp"
#include "VectorRepository.hpp"
#include <boost/optional.hpp>

class Tetrahedron
{
private:
	std::vector<VectorRef> _vertices;
	mutable boost::optional<Vector3D> _center;   // Caches the center, which doesn't really change the object
	mutable boost::optional<double> _volume;     // Another cache
	mutable boost::optional<double> _radius;
	mutable boost::optional<double> _radius2;
	mutable boost::optional<Vector3D> _centerOfMass;

	Vector3D CalculateCenter() const;
	double CalculateVolume() const;
	double CalculateRadius() const;
	double CalculateRadius2() const;
	Vector3D CalculateCenterOfMass() const;

public:
	Tetrahedron() { }  // Empty default constructor, for putting tetrahedra in maps
	Tetrahedron(const std::vector<VectorRef> &vertices);
	Tetrahedron(const VectorRef v1, const VectorRef v2, const VectorRef v3, const VectorRef v4);

	Vector3D center() const;
	double volume() const;
	double radius() const;
	double radius2() const; // Radius**2
	Vector3D centerOfMass() const;

	const vector<VectorRef>& vertices() const { return _vertices; }
	const VectorRef& operator[](size_t index) const { return _vertices[index]; }
};

std::ostream& operator<<(std::ostream &output, const Tetrahedron &tetrahedron);

#endif // TETRAHEDRON_HPP