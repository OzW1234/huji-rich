//\file GhostBusters.cpp
//\brief Implementation of the common Ghost Busters
//\author Itay Zandbank

#include "GhostBusters.hpp"
#include <set>
#include <unordered_set>

using namespace std;

static const double GHOST_BOUNDARY = 1e-5;

static set<Subcube> GetRigidWallSubcubes()
{
	const char *offsets[] = { "-  ", "+  ", " - ", " + ", "  -", "  +" };
	set<Subcube> subcubes;
	for (int i = 0; i < 6; i++)
		subcubes.insert(Subcube(offsets[i]));

	return subcubes;
}
static set<Subcube> _rigidWallSubcubes = GetRigidWallSubcubes();

GhostBuster::GhostMap BruteForceGhostBuster::operator()(const Delaunay &del, const OuterBoundary3D &boundary) const
{
	GhostMap ghosts;
	for (vector<VectorRef>::const_iterator itV = del.InputPoints().begin(); itV != del.InputPoints().end(); itV++)
	{
		GhostMap::mapped_type *ghostSet = NULL;
		for (set<Subcube>::const_iterator itS = _rigidWallSubcubes.begin(); itS != _rigidWallSubcubes.end(); itS++)
		{
			if (!ghostSet)
			{
				ghosts[*itV] = GhostMap::mapped_type();
				ghostSet = &ghosts[*itV];
			}
			Vector3D ghost = boundary.reflect(**itV, *itS);
			ghostSet->insert(GhostPoint(*itS, ghost));
		}
	}

	return ghosts;
}

GhostBuster::GhostMap FullBruteForceGhostBuster::operator()(const Delaunay &del, const OuterBoundary3D &boundary) const
{
	GhostMap ghosts;
	for (vector<VectorRef>::const_iterator itV = del.InputPoints().begin(); itV != del.InputPoints().end(); itV++)
	{
		GhostMap::mapped_type *ghostSet = NULL;
		for (set<Subcube>::const_iterator itS = _rigidWallSubcubes.begin(); itS != _rigidWallSubcubes.end(); itS++)
		{
			if (!ghostSet)
			{
				ghosts[*itV] = GhostMap::mapped_type();
				ghostSet = &ghosts[*itV];
			}
			Vector3D ghost = boundary.reflect(**itV, *itS);
			ghostSet->insert(GhostPoint(*itS, ghost));
		}
	}

	return ghosts;
}

GhostBuster::GhostMap RigidWallGhostBuster::operator()(const Delaunay &del, const OuterBoundary3D &boundary) const
{
	unordered_set<size_t> outer = FindOuterTetrahedra(del);
	unordered_set<size_t> edge = FindEdgeTetrahedra(del, outer);
	breach_map breaches = FindHullBreaches(del, edge, outer, boundary);

	GhostMap ghosts;

	if (breaches.empty())
		return ghosts;

	for (breach_map::iterator it = breaches.begin(); it != breaches.end(); it++)
	{
		GhostMap::mapped_type *ghostSet = NULL;

		VectorRef pt = it->first;
		for (unordered_set<Subcube>::iterator itSubcube = it->second.begin(); itSubcube != it->second.end(); itSubcube++)
		{
			if (!ghostSet)
			{
				ghosts[pt] = GhostMap::mapped_type();
				ghostSet = &ghosts[pt];
			}
			Vector3D ghost = GetGhostPoint(boundary, *pt, *itSubcube);
			ghostSet->insert(GhostPoint(*itSubcube, ghost));
		}
	}

	return ghosts;
}

RigidWallGhostBuster::breach_map RigidWallGhostBuster::FindHullBreaches(const Delaunay &del, const unordered_set<size_t>& edgeTetrahedra,
	const unordered_set<size_t> &outerTetrahedra, const OuterBoundary3D &boundary) const
{
	breach_map result;
	unordered_set<size_t> candidateTetrahedra(edgeTetrahedra.begin(), edgeTetrahedra.end());
	while (!candidateTetrahedra.empty())
	{
		unordered_set<size_t> nextCandidates;

		for (unordered_set<size_t>::iterator itTetra = candidateTetrahedra.begin(); itTetra != candidateTetrahedra.end(); itTetra++)
		{
			const Tetrahedron &t = del[*itTetra];
			for (int iv = 0; iv < 4; iv++)
			{
				VectorRef pt = t[iv];
				if (del.IsBigTetrahedron(pt))
					continue;

				vector<size_t> tetrahedraIndices = del.VertexNeighbors(pt);
				unordered_set<Subcube> breaches;
				if (result.find(pt) != result.end())
					breaches = result[pt];

				for (vector<size_t>::iterator itTetrahedron = tetrahedraIndices.begin(); itTetrahedron != tetrahedraIndices.end(); itTetrahedron++)
				{
					/*if (contains(outerTetrahedra, *itTetrahedron))
						continue; */

					const Tetrahedron &tetrahedron = del[*itTetrahedron];
					bool breaching = false;
					const set<Subcube> &subcubes = GetAllSubcubes();
					for (set<Subcube>::const_iterator itSubcube = subcubes.begin(); itSubcube != subcubes.end(); itSubcube++)
					{
						if (boundary.distance(tetrahedron.center(), *itSubcube) < tetrahedron.radius())
						{
							if (breaches.find(*itSubcube) == breaches.end())
							{
								breaches.insert(*itSubcube);
								breaching = true;
							}
						}
					}
					if (breaching)
					{
						// If a tetrahedron is breaching, we need to test all its points and all the points of the tetrahedra touching pt
						nextCandidates.insert(*itTetrahedron);
						const vector<size_t> &neighbors = del.VertexNeighbors(pt);
						// const vector<size_t> &neighbors = del.TetrahedraNeighbors(*itTetrahedron);
						// const vector<size_t> &neighbors = del.VertexNeighbors(pt);
						nextCandidates.insert(neighbors.begin(), neighbors.end());
					}
				}
				result[pt] = breaches;
			}
		}

		candidateTetrahedra = nextCandidates;
	}

	return result;
}

//\brief Find all the Outer Tetrahedra
//\remark An Outer Tetrahedron is a tetrahedron that has a vertex in the big tetrahedron
unordered_set<size_t> RigidWallGhostBuster::FindOuterTetrahedra(const Delaunay &del) const
{
	unordered_set<size_t> result;

	for (int i = 0; i < 4; i++)
	{
		VectorRef pt = del.BigTetrahedron()[i];
		vector<size_t> tetrahedra = del.VertexNeighbors(pt);
		result.insert(tetrahedra.begin(), tetrahedra.end());
	}

	return result;
}

//\brief Find all the Edge Tetrahedra
//\remark An Edge Tetrahedron has a vertex that belongs to an Outer Tetrahedron. Edge Tetrahedra are not Outer Tetrahedra
unordered_set<size_t> RigidWallGhostBuster::FindEdgeTetrahedra(const Delaunay &del, const unordered_set<size_t>& outerTetrahedra) const
{
	unordered_set<size_t> edgeTetrahedra;

	// Go over all the outer tetrahedra
	for (unordered_set<size_t>::const_iterator it = outerTetrahedra.begin(); it != outerTetrahedra.end(); it++)
	{
		Tetrahedron t = del[*it];
		// And all their vertices
		for (int i = 0; i < 4; i++)
		{
			VectorRef pt = t[i];
			if (del.IsBigTetrahedron(pt))  // Ignore the BigTetrahedron - all tetrahedra touching this vertex are Outer and not Edge
				continue;
			// Find all the neighbor tetrahedra of pt
			vector<size_t> tetrahedra = del.VertexNeighbors(pt);
			// Add them to the set
			for (vector<size_t>::iterator ptIt = tetrahedra.begin(); ptIt != tetrahedra.end(); ptIt++)
				if (!contains(outerTetrahedra, *ptIt))  // But only if their not Outer
					edgeTetrahedra.insert(*ptIt);
		}
	}

	return edgeTetrahedra;
}

Vector3D RigidWallGhostBuster::GetGhostPoint(const OuterBoundary3D &boundary, const Vector3D pt, const Subcube subcube) const
{
	return boundary.reflect(pt, subcube);
}

const set<Subcube> &RigidWallGhostBuster::GetAllSubcubes() const
{
	return _rigidWallSubcubes;
}


Vector3D PeriodicGhostBuster::GetGhostPoint(const OuterBoundary3D &boundary, const Vector3D pt, const Subcube subcube) const
{
	return boundary.copy(pt, subcube);
}

const set<Subcube> &PeriodicGhostBuster::GetAllSubcubes() const
{
	return Subcube::all();
}
