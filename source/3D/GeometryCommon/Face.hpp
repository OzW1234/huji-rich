/*! \file Face.hpp
  \brief Face between cells
  \author Elad Steinberg
*/

#ifndef FACE_HPP
#define FACE_HPP 1

#include <vector>
#include "VectorRepository.hpp"
#include <boost/optional.hpp>
#include <iostream>

using std::size_t;

//! \brief Interface between two cells
class Face
{
	// TODO: Make Face immutable - created with the neighbors and that's it.
private:
	boost::optional<size_t> _neighbor1, _neighbor2;
	mutable boost::optional<double> _area; // Mutable because these are cached
	mutable boost::optional<Vector3D> _centroid;

	void CalculateArea() const;
	void CalculateCentroid() const;

public:
	static const Face Empty;   // An empty face

	//! \brief Points at the ends of the edge
	const std::vector<VectorRef> vertices;

	Face(const std::vector<VectorRef> &verts, boost::optional<size_t> neighbor1=boost::none, boost::optional<size_t> neighbor2=boost::none);
	Face(const Face &other);

	size_t NumNeighbors() const { return (_neighbor1.is_initialized() ? 1 : 0) + (_neighbor2.is_initialized() ? 1 : 0); }
	boost::optional<size_t> Neighbor1() const 
	{
		return _neighbor1;
	}
	boost::optional<size_t> Neighbor2() const
	{
		return _neighbor2;
	}

	boost::optional<size_t> OtherNeighbor(int cell)
	{
		if (Neighbor1().is_initialized() && *Neighbor1() == cell)
			return Neighbor2();
		return Neighbor1();
	}
	
	/*! \brief Returns the area of the face
	\return Length
	*/
	double GetArea() const;

	/*! \brief Returns the centroid of the face
	\return The centroid
	\remarks The centroid is the weighted average of the centroid of all the triangles forming the face - the weight being the triangle's area */
	Vector3D GetCentroid() const;

	/*! \brief Reorders the vertices based on their angle from the center
	\returns A new face - reordered
	\remark This is useful when the face order wasn't constructed properly
	\remark It is assumed the vertices are all coplanar
	\remark It is assumed the vertices form a convex polygon
	*/
	Face ReorderVertices();

	/*! \brief Sees if the face is identical to a list of vertices
	\param List of vertices
	\return True if this is the same face
	\remark
	    Two faces are identical if they contain the same vertices in the same order. The faces need not start
		with the same vertex, and also don't need to be the same direction, so:
		ABCD is identical to BCDA and also to CBAD but not to BACD
	*/
	bool IdenticalTo(const vector<VectorRef> &vertices) const;

private:
	/*! \brief Returns the angle between the two vectors.
	\param v1 First vector
	\param v2 Second vector
	\returns The angle between the vectors, between 0 and 2*Pi
	\remark The angle between 0 and Pi is calculated using the dot product of v1 and v2.
	  Then v1xv2 is consulted to determine if the angle is between 0 and Pi or between Pi and 2*Pi.
	  By convention, the first non-zero element of the cross product is consulted. If it's positive,
	  the angle is said to be between 0 and Pi, if it's negative it's said to be between Pi and 2*Pi
	\remark This function is used solely for ordering the faces, so this implementation is enough
	*/
	static double FullAngle(const Vector3D &v1, const Vector3D &v2);
};

/*! \brief Calculates the centroid of aa face
  \param face The face
  \return Centroid
 */
Vector3D calc_centroid(const Face& face);

template <typename T>
std::ostream& operator<<(std::ostream& output, const boost::optional<T>& opt)
{
	if (opt)
		output << *opt;
	else
		output << "NONE";
	return output;
}

// \brief Compares to faces.
// \returns True if the faces are identical, meaning they have exactly the same vertices, regardless of ordering.
// \remarks Since faces are convex, they are the same iff they have the same vertices.
bool operator==(const Face &face1, const Face &face2);

namespace std
{
	template<>
	struct hash<Face>
	{
		typedef Face argument_type;
		typedef size_t result_type;

		result_type operator()(const argument_type &face) const
		{
			size_t value;
			std::hash<VectorRef> hasher;
			for (size_t i = 0; i < face.vertices.size(); i++)
				value ^= hasher(face.vertices[i]);  // Ignores the vertex ordering, as it should

			return value;
		}
	};
}

#endif	// FACE_HPP
