//!\file VectorRepository.hpp
//!\brief A repository of vectors, allowing the use of vector ids instead of the entire vector
//!\author Itay Zandbank

#ifndef VECTOR_REPOSITORY_HPP
#define VECTOR_REPOSITORY_HPP

#include "Vector3D.hpp"
#include <iostream>

struct VectorRefHasher;

//! \brief A Repository of Vectors
//! \remark
//!		The repository is used to give each 3D vector a single hashable ID.
class VectorRef
{
private:
	size_t _id;

public:
	//! \brief Creates a VectorRef from a Vector3D
	//! \param vector The Vector3D
	//! \returns The VectorRef.
	//! \remarks Calling VectorRef with the same vector twice will return the same reference.
	VectorRef(const Vector3D &vector);

	//! \brief Copy constructor
	VectorRef(const VectorRef &other);

	//! \brief Default constructor
	//! \remarks For C++03 container compliance.
	VectorRef();

	const Vector3D *operator->() const;  //!< Access the underlying Vector3D
	const Vector3D &operator*() const;   //!< Access the underlying Vector3D

	friend struct VectorRefHasher ;
	friend bool operator==(const VectorRef &v1, const VectorRef &v2);
	friend bool operator<(const VectorRef &v1, const VectorRef &v2);

	//! \brief Assignment operator
	//! \param other VectorRef
	VectorRef& operator=(const VectorRef &other);

	//! \brief Converts a vector of Vector3D to a vector of VectorRefs
	//! \param points The Vector3D
	//! \returns A vector of VectorRefs corresponding to the input vectors
	static std::vector<VectorRef> vector(const std::vector<Vector3D> &points);
};

//! \brief A hasher for VectorRef
//! \remarks We can't use std::hash because we're C++03 compliant.
struct VectorRefHasher
{
	typedef VectorRef argument_type;
	typedef size_t result_type;

	result_type operator()(const argument_type &ref) const
	{
		return ref._id;
	}
};

//! \brief Output a VectorRef
std::ostream& operator<<(std::ostream& output, const VectorRef &vref);

//! \brief Check two VectorRefs for equality
bool operator==(const VectorRef &v1, const VectorRef &v2);

//! \brief Check two VectorRefs for inequality
bool operator!=(const VectorRef &v1, const VectorRef &v2);

//! \brief Compare two VectorRefs
bool operator<(const VectorRef &v1, const VectorRef &v2);

#endif