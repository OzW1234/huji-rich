//\file GhostBusters.hpp
//\brief Contains the classes that generate and deal with ghost points
//\author Itay Zandbank

#ifndef GHOST_BUSTER_HPP
#define GHOST_BUSTER_HPP 1

#include <set>
#include <vector>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include "../GeometryCommon/Vector3D.hpp"
#include "../GeometryCommon/OuterBoundary3D.hpp"
#include "../GeometryCommon/VectorRepository.hpp"
#include "Delaunay.hpp"

//\brief The abstract GhostBuster class. A GhostBuster class is really just a Functor object.
class GhostBuster
{
public:
	//! \brief A GhostPoint - including its subcube.
	typedef std::pair<Subcube, VectorRef> GhostPoint;
	//! \brief Calculates the hash of the Ghost Point
	struct GhostPointHasher
	{
		size_t operator()(const GhostPoint &gp) const
		{
			SubcubeHasher subcubeHash;
			VectorRefHasher vectorHash;

			return subcubeHash(gp.first) ^ vectorHash(gp.second);
		}
	};
	//! \brief A mapping between points and their ghost points.
	typedef boost::unordered::unordered_map<VectorRef, boost::unordered::unordered_set<GhostPoint, GhostPointHasher>, VectorRefHasher> GhostMap;

	//! \brief Calculates the Ghost Points
	//! \param del The Delaunay Triangulation
	//! \param boundary The 3D boundary box
	//! \return The Ghost points in a pt->{ghost points} map, each point and its ghost points.
	virtual GhostMap operator()(const Delaunay &del, const OuterBoundary3D &boundary) const = 0;
};

//! \brief A simple implementation that copies each point 6 times
class BruteForceGhostBuster : public GhostBuster
{
public:
	virtual GhostMap operator()(const Delaunay &del, const OuterBoundary3D &boundary) const;
};

//! \brief A simple implementation that duplicates each point 26 times
//! \remark This should only be used in testing, there's no reason to use this and not BruteForceGhostBuster.
class FullBruteForceGhostBuster : public GhostBuster
{
public:
	virtual GhostMap operator()(const Delaunay &del, const OuterBoundary3D &boundary) const;
};

//! \brief An efficient algorithm that implements a "rigid-wall" boundary by duplicating just the necessary points
class RigidWallGhostBuster : public GhostBuster
{
public:
	virtual GhostMap operator()(const Delaunay &del, const OuterBoundary3D &boundary) const;

protected:
	//! \brief Ghost a point into a specific subcube
	//! \param boundary The 3D box surrounding the points
	//! \param pt The point to ghost
	//! \param subcube The subcube of the new ghost point
	virtual Vector3D GetGhostPoint(const OuterBoundary3D &boundary, const Vector3D pt, const Subcube subcube) const;

	//! \brief Returns all the Subcubes that are relevant for this Ghost Buster.
	virtual const std::set<Subcube> &GetAllSubcubes() const;

private:
	boost::unordered::unordered_set<size_t> FindOuterTetrahedra(const Delaunay &del) const ;
	boost::unordered::unordered_set<size_t> FindEdgeTetrahedra(const Delaunay &del, const boost::unordered::unordered_set<size_t>& outerTetrahedra) const;

	typedef boost::unordered::unordered_map<VectorRef, boost::unordered::unordered_set<Subcube, SubcubeHasher>, VectorRefHasher> breach_map;
	breach_map FindHullBreaches(const Delaunay &del, 
		const boost::unordered::unordered_set<size_t>& edgeTetrahedra,
		const OuterBoundary3D &boundary) const;

	template<typename T>
	static bool contains(const boost::unordered::unordered_set<T> &set, const T &val)
	{
		return set.find(val) != set.end();
	}
};

//! \brief An implementation that creates a period boundary
class PeriodicGhostBuster : public RigidWallGhostBuster
{
protected:
	virtual Vector3D GetGhostPoint(const OuterBoundary3D &boundary, const Vector3D pt, const Subcube subcube) const;
	virtual const std::set<Subcube> &GetAllSubcubes() const;
};

#endif