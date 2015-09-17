//! \file Subcube.hpp
//! \brief The Subcube concept
//! \author Itay Zandbank

#ifndef SUBCUBE_H
#define SUBCUBE_H

#include "../Utilities/assert.hpp"
#include "../GeometryCommon/Vector3D.hpp"
#include <set>

//! \brief A Subcube
//! \remarks
//!     The 3D space contains one main "subcube" defined by the boundary (it's not really a cube, but it's easy to think of it as one)
//!     and 26 adjacent "subcubes" - totaling in 27 subcubes (3 in each axis)
//!     A subcube is determined by its three offsets - one per axis. An offset can be '-', ' ' or '+' for minus, center and plus respectively
class Subcube
{
private:
	char _offsets[3];

	static size_t Num(char c)
	{
		switch (c)
		{
		case MINUS:
			return 0;
		case CENTER:
			return 1;
		case PLUS:
			return 2;
		}
		BOOST_ASSERT(false); // This value is illegal
		return 9999;
	}

	static std::set<Subcube> _all;

public:
	//! \brief Constructs a subcube from its offset ('---' to '+++')
	Subcube(const char offsets[3]);

	//! \brief Returns the i'th character of the offset
	char operator[](int i) const { return _offsets[i]; }

	const static char MINUS = '-';
	const static char CENTER = ' ';
	const static char PLUS = '+';

	//! \brief Converts the subcube into a hashable number
	size_t Num() const
	{
		return Num(_offsets[0]) * 9 + Num(_offsets[1]) * 3 + Num(_offsets[2]);
	}

	//! \brief Returns the number of offsets that aren't in the center
	//! \remark For example, offset '---' has 3 non-centers, while '-+ ' has two non-centers.
	int NonCenters() const
	{
		return (int)(_offsets[0] != CENTER) + (int)(_offsets[1] != CENTER) + (int)(_offsets[2] != CENTER);
	}

	//! \brief Returns all the subcubes (without the main cube '   ')
	static const std::set<Subcube>& all();

	friend std::ostream& operator<<(std::ostream& output, const Subcube &sc);
};

//! \brief Compare two subcubces
bool operator<(const Subcube &sc1, const Subcube &sc2);

//! \brief Compare two subcubces
bool operator==(const Subcube &sc1, const Subcube &sc2);

//! \brief Outputs a subcube
std::ostream& operator<<(std::ostream& output, const Subcube &sc);

//! \brief Hash function object for the Subcube
//! \remark We're not using std::hash<Subcube> because this code needs to be C++03 compliant.
struct SubcubeHasher
{
	typedef Subcube argument_type;
	typedef std::size_t result_type;

	result_type operator()(const argument_type &sc) const
	{
		return sc.Num();
	}
};

#endif
