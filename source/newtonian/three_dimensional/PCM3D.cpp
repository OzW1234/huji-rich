#include "PCM3D.hpp"

PCM3D::PCM3D(Ghost3D const& ghost) :ghost_(ghost) {}

void PCM3D::operator()(const Tessellation3D& tess, const vector<ComputationalCell3D>& cells, double time,
	vector<pair<ComputationalCell3D, ComputationalCell3D> > &res, TracerStickerNames const& tracerstickersnames)const
{
	size_t Nfaces = tess.GetTotalFacesNumber();
	size_t Npoints = tess.GetPointNo();
	res.resize(Nfaces);
	boost::container::flat_map<size_t, ComputationalCell3D> ghosts = ghost_(tess, cells, time, tracerstickersnames);
	for (size_t i = 0; i < Nfaces; ++i)
	{
		size_t n0 = tess.GetFaceNeighbors(i).first;
		if (n0 < Npoints)
			res[i].first = cells[n0];
		else
			if (ghosts.find(n0) == ghosts.end())
				res[i].first = cells.at(n0);
			else
				res[i].first = ghosts.at(n0);
		size_t n1 = tess.GetFaceNeighbors(i).second;
		if (n1 < Npoints)
			res[i].second = cells[n1];
		else
			if (ghosts.find(n1) == ghosts.end())
				res[i].first = cells.at(n1);
			else
				res[i].first = ghosts.at(n1);
	}
}