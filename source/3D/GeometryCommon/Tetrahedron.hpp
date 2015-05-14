/*! \file Tetrahedron.hpp
 \brief A simple encapsulation of a tetrahedron 
 \author Itay Zandbank
*/

#ifndef TETRAHEDRON_HPP
#define TETRAHEDRON_HPP

#include <vector>
#include "Vector3D.hpp"
#include "VectorRepository.hpp"
#include <boost/optional.hpp>

//! \brief A Simple 3D Tetrahedron
class Tetrahedron
{
private:
	std::vector<VectorRef> _vertices;
	mutable boost::optional<Vector3D> _center;   // Caches the center, which doesn't really change the object
	mutable boost::optional<double> _volume;     // Another cache
	mutable boost::optional<double> _radius2;
	mutable boost::optional<Vector3D> _centerOfMass;

	Vector3D CalculateCenter() const;
	double CalculateVolume() const;
	double CalculateRadius2() const;
	Vector3D CalculateCenterOfMass() const;

public:
	//! \param verts The Tetrahedron vertices
	Tetrahedron(const std::vector<VectorRef> &verts);

	//! \param v1 Vertex number 1
	//! \param v2 Vertex number 2
	//! \param v3 Vertex number 3
	//! \param v4 Vertex number 4
	Tetrahedron(const VectorRef v1, const VectorRef v2, const VectorRef v3, const VectorRef v4);

	Tetrahedron(const Tetrahedron &other);

	Vector3D center() const; //!< The Tetrahedron's center
	double volume() const;  //!< The Tetrahedron's volume
	double radius2() const; //!< The Tetrahedron's Radius^2
	Vector3D centerOfMass() const;  //!< The Tetrahedron's Center of Mass

	const vector<VectorRef>& vertices() const { return _vertices; }  //!< The Tetrahedron's vertices
	const VectorRef& operator[](size_t index) const { return _vertices[index]; }  //!< Access to a specific vertex
};

//!\brief Output a tetrahedron.
std::ostream& operator<<(std::ostream &output, const Tetrahedron &tetrahedron);

#endif // TETRAHEDRON_HPP