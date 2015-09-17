/*! \file Face.hpp
  \brief A Voronoi cell's face.
  \author Itay Zandbank
*/

#ifndef FACE_HPP
#define FACE_HPP 1

#include <vector>
#include "VectorRepository.hpp"
#include <boost/optional.hpp>
#include <iostream>

using std::size_t;

//! \brief A face of a Voronoi Cell
//! \remark A Voronoi Face has two neighboring cells.
//! \remarks This class is immutable
class Face
{
private:
	boost::optional<size_t> _neighbor1, _neighbor2;
	mutable boost::optional<double> _area; // Mutable because these are cached
	mutable boost::optional<Vector3D> _centroid;

	void CalculateArea() const;
	void CalculateCentroid() const;

public:
	static const Face Empty;   //!< An empty face

#ifdef STRICT_CPP03
	// C++03's std::vector requires types to be CopyAssignable if they are to be added to a vector.
	// Face is immutable, and thus not CopyAssignable.
	// If this macro is defined, the type is no lone immutable and hence CopyAssignable.
	// Do not define this flag with C++11 - making Face mutable can lead to bugs if someone
	// modifies vertices from the outside

	//! \brief The vertices of the face - in order
	//! \remark This is the C++03 compliant version - where vertices is not const. It should be const, but C++03 requires CopyAssignable classes to be put in vectors.
	std::vector<VectorRef> vertices;
	Face() : _neighbor1(), _neighbor2(), _area(), _centroid(), vertices() { }
#else
	//! \brief The vertices of the face - in order
	const std::vector<VectorRef> vertices;
#endif

	//! \brief Constructs a new face.
	//! \param verts The face's vertices
	//! \param neighbor1 The face's first neighbor (number of neighbor cell)
	//! \param neighbor2 The face's second neighbor (number of neighbor cell)
	//! \remarks A cell used for intermediate calculations may have no neighbors
	Face(const std::vector<VectorRef> &verts, boost::optional<size_t> neighbor1 = boost::none, boost::optional<size_t> neighbor2 = boost::none);

	//! \brief Copy constructor
	Face(const Face &other);

	//! \brief Number of non-empty neighbors
	size_t NumNeighbors() const { return (_neighbor1.is_initialized() ? 1 : 0) + (_neighbor2.is_initialized() ? 1 : 0); }

	//! \brief The first neighbor (number of neigbor cell)
	boost::optional<size_t> Neighbor1() const 
	{
		return _neighbor1;
	}

	//! \brief The second neighbor (number of neighbor cell)
	boost::optional<size_t> Neighbor2() const
	{
		return _neighbor2;
	}

	//! \brief Returns the Other Neighbor
	//! \param cell The number of one neighbor
	//! \return The other face's neighbor (not cell)
	boost::optional<size_t> OtherNeighbor(size_t cell) const
	{
		if (Neighbor1().is_initialized() && *Neighbor1() == cell)
			return Neighbor2();
		return Neighbor1();
	}
	
	//! \brief Returns the area of the face
	//! \return The area
	double GetArea() const;

	//! \brief Returns the centroid of the face
	//! \return The centroid
	//! \remarks The centroid is the weighted average of the centroid of all the triangles forming the face - the weight being the triangle's area */
	Vector3D GetCentroid() const;
};

/*! \brief Calculates the centroid of aa face
  \param face The face
  \return Centroid
 */
Vector3D calc_centroid(const Face& face);

//! \brief Output a face
template <typename T>
std::ostream& operator<<(std::ostream& output, const boost::optional<T>& opt)
{
	if (opt)
		output << *opt;
	else
		output << "NONE";
	return output;
}

//! \brief Compares to faces.
//! \returns True if the faces are identical, meaning they have exactly the same vertices, regardless of ordering.
//! \remarks Since faces are convex, they are the same iff they have the same vertices.
bool operator==(const Face &face1, const Face &face2);

#endif	// FACE_HPP
