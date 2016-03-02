/*! \file PCM.hpp
\brief Class for pcm interpolation of the hydrodynamic variables
\author Elad Steinberg
*/

#ifndef PCM_HPP
#define PCM_HPP 1

#include "../spatial_reconstruction.hpp"
#include "../../common/hydrodynamic_variables.hpp"
#include "../../../tessellation/tessellation.hpp"
#include "../computational_cell_2d.hpp"
#include "../GhostPointGenerator.hpp"

class PCM : public SpatialReconstruction
{
private:
	GhostPointGenerator const & ghost_;
public:
	explicit PCM(GhostPointGenerator const& ghost);

	void operator()
		(const Tessellation& tess,
			const vector<ComputationalCell>& cells,
			double time, vector<pair<ComputationalCell, ComputationalCell> > &res,
			TracerStickerNames const& tracerstikersnames) const;
};

#endif // SPATIAL_RECONSTRUCTION_HPP
