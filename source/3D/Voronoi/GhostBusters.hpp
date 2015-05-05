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
	typedef std::pair<Subcube, VectorRef> GhostPoint;
	struct GhostPointHasher
	{
		size_t operator()(const GhostPoint &gp) const
		{
			SubcubeHasher subcubeHash;
			VectorRefHasher vectorHash;

			return subcubeHash(gp.first) ^ vectorHash(gp.second);
		}
	};
	typedef boost::unordered::unordered_map<VectorRef, boost::unordered::unordered_set<GhostPoint, GhostPointHasher>, VectorRefHasher> GhostMap;

	virtual GhostMap operator()(const Delaunay &del, const OuterBoundary3D &boundary) const = 0;
};

//\brief A simple implementation that copies each point 26 times
class BruteForceGhostBuster : public GhostBuster
{
public:
	virtual GhostMap operator()(const Delaunay &del, const OuterBoundary3D &boundary) const;
};

class FullBruteForceGhostBuster : public GhostBuster
{
public:
	virtual GhostMap operator()(const Delaunay &del, const OuterBoundary3D &boundary) const;
};

//\brief An implementation that checks only points that are on edge-faces.
class RigidWallGhostBuster : public GhostBuster
{
public:
	virtual GhostMap operator()(const Delaunay &del, const OuterBoundary3D &boundary) const;

protected:
	virtual Vector3D GetGhostPoint(const OuterBoundary3D &boundary, const Vector3D pt, const Subcube subcube) const;
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

//\brief An implementation that creates a period boundary
class PeriodicGhostBuster : public RigidWallGhostBuster
{
protected:
	virtual Vector3D GetGhostPoint(const OuterBoundary3D &boundary, const Vector3D pt, const Subcube subcube) const;
	virtual const std::set<Subcube> &GetAllSubcubes() const;
};

#endif