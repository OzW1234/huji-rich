//\file VectorRepository.cpp
//\brief Implementation of the Vector Repository
//\author Itay Zandbank

#include "Vector3D.hpp"
#include "VectorRepository.hpp"
#include "../Utilities/assert.hpp"

#include <map>

using namespace std;

class VectorRepository
{
private:
	typedef map<Vector3D, size_t> map_type;
	map_type _vectorMap;
	vector<Vector3D> _repository;

public:
	VectorRepository() : _vectorMap(), _repository()
	{
	}
	size_t GetVectorId(const Vector3D &vec);
	const Vector3D &GetVector(size_t index) const;
};

size_t VectorRepository::GetVectorId(const Vector3D &vec)
{
	map_type::iterator it = _vectorMap.find(vec);
	if (it != _vectorMap.end())
		return it->second;

	_repository.push_back(vec);
	size_t index = _repository.size() - 1;
	_vectorMap[vec] = index;
	return index;
}

const Vector3D& VectorRepository::GetVector(size_t index) const
{
	BOOST_ASSERT(index < _repository.size());
	return _repository[index];
}

static VectorRepository theRepository;

VectorRef::VectorRef(const Vector3D &vec) : _id(theRepository.GetVectorId(vec))
{
}

VectorRef::VectorRef() : _id((size_t)-1)
{
}

VectorRef::VectorRef(const VectorRef &other) : _id(other._id)
{
}

std::vector<VectorRef> VectorRef::vector(const std::vector<Vector3D> &points)
{
	std::vector<VectorRef> output;
	output.reserve(points.size());
	for (std::vector<Vector3D>::const_iterator it = points.begin(); it != points.end(); it++)
		output.push_back(*it);

	return output;
}


VectorRef& VectorRef::operator=(const VectorRef &other)
{
	_id = other._id;
	return *this;
}

const Vector3D &VectorRef::operator*() const
{
	return theRepository.GetVector(_id);
}

const Vector3D *VectorRef::operator->() const
{
	return &theRepository.GetVector(_id);
}

ostream& operator<<(ostream& output, const VectorRef &vref)
{
	output << *vref;
	return output;
}

bool operator==(const VectorRef &v1, const VectorRef &v2)
{
	return v1._id == v2._id;
}

bool operator!=(const VectorRef &v1, const VectorRef &v2)
{
	return !(v1==v2);
}

bool operator<(const VectorRef &v1, const VectorRef &v2)
{
	return v1._id < v2._id;
}
