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

	boost::optional<size_t> OtherNeighbor(size_t cell) const
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
