/*! \file OuterBoundary3D.hpp
  \brief Outer Boundary Conditions
  \author Itay Zandbank
 */

#ifndef OUTERBOUNDARY3D_HPP
#define OUTERBOUNDARY3D_HPP 1

#include "Subcube.hpp"
//#include "Tessellation3D.hpp"

//! \brief Class describing the boundry box of the Voronoi Tessallation.
class OuterBoundary3D
{
private:
	Vector3D _frontUpperRight, _backLowerLeft;
	Vector3D _dimensions;

public:
	//! \brief Constructs a Boundray instance
	//! \param frontUpperRight Front Upper Right coordinate of bounding box.
	//! \param backLowerLeft Back Lower Left coordinate of bounding box
	OuterBoundary3D(Vector3D frontUpperRight, Vector3D backLowerLeft);
	
	//! \brief the Front Upper Right corner of the box
	const Vector3D &FrontUpperRight() const { return _frontUpperRight; }

	//! \brief the Back Lower Left corner of the box
	const Vector3D &BackLowerLeft() const { return _backLowerLeft; }

	//! \brief Returns the distance from the point, taking the subcube into account
	//! \param pt The point
	//! \param subcube The subcube - from '---' to '+++'.
	double distance(const Vector3D &pt, const Subcube subcube) const;

	//! \brief Returns the square of the distance from the point, taking the subcube into account
	//! \param pt The point
	//! \param subcube The subcube - from '---' to '+++'.
	//! \remarks distance2(pt, subcube) = distance(pt, subcube) ** 2
	double distance2(const Vector3D &pt, const Subcube subcube) const;


	//!\brief returns the vector to the subcube. The vector is the shortest to that subcube.
	//! \param pt The Point
	//! \param subcube The subcube
	//! \returns The vector. The vector's size is distance(pt, subcube)
	Vector3D vector(const Vector3D &pt, const Subcube subcube) const;

	//! \brief Reflects a point into a subcube
	//! \param pt The Point
	//! \param subcube The Subcube
	//! \returns Pt reflected through the boundary into the subcube
	virtual Vector3D reflect(const Vector3D &pt, const Subcube subcube) const;

	//! \brief Copies a point to a subcube
	//! \param pt The Point
	//! \param subcube The Subcube
	//! \returns A copy of Pt inside subcube. It will be exactly the same distance from the subcube's origin
	virtual Vector3D copy(const Vector3D &pt, const Subcube subcube) const;

	//! \brief Checks if a point is inside the boundary
	//! \param pt Point
	//! \returns true if the point is inside the boundary
	//! \remarks Points on the boundary are considered inside
	bool inside(const Vector3D &pt) const;

private:
	Vector3D vector_face(const Vector3D &pt, const Subcube subcube) const;
	Vector3D vector_edge(const Vector3D &pt, const Subcube subcube) const;
	Vector3D vector_point(const Vector3D &pt, const Subcube subcube) const;
};
#endif // OUTERBOUNDARY3D_HPP
