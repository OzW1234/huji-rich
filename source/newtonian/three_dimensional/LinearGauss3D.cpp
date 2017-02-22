#include "LinearGauss3D.hpp"
#include "../../misc/utils.hpp"
#include <boost/array.hpp>
#ifdef RICH_MPI
#include "../../mpi/mpi_commands.hpp"
#endif

namespace
{
	void CheckCell(ComputationalCell3D const& cell)
	{
		if (cell.density < 0 || cell.pressure < 0)
			throw UniversalError("Bad cell after interpolation in LinearGauss3D");
	}

	void GetNeighborMesh(Tessellation3D const& tess, size_t cell_index,vector<Vector3D> &res, vector<size_t> const& faces)
	{
		res.resize(faces.size());
		const size_t nloop = res.size();
		for (size_t i = 0; i < nloop; ++i)
		{
			if(tess.GetFaceNeighbors(faces[i]).first==cell_index)
				res[i] = tess.GetMeshPoint(tess.GetFaceNeighbors(faces[i]).second);
			else
				res[i] = tess.GetMeshPoint(tess.GetFaceNeighbors(faces[i]).first);
		}
	}

	void GetNeighborCM(Tessellation3D const& tess, size_t cell_index,vector<Vector3D> &res,vector<size_t> const& faces)
	{
		res.resize(faces.size());
		const size_t nloop = faces.size();
		for (size_t i = 0; i < nloop; ++i)
		{
			if (tess.GetFaceNeighbors(faces[i]).first == cell_index)
				res[i] = tess.GetCellCM(tess.GetFaceNeighbors(faces[i]).second);
			else
				res[i] = tess.GetCellCM(tess.GetFaceNeighbors(faces[i]).first);
		}
	}

	vector<ComputationalCell3D const*> GetNeighborCells(Tessellation3D const& tess, size_t cell_index,
		vector<ComputationalCell3D> const& cells, vector<size_t> const& faces)
	{
		vector<ComputationalCell3D const*> res(faces.size());
		const size_t nloop = faces.size();
		for (size_t i = 0; i < nloop; ++i)
		{
			size_t other_cell = (tess.GetFaceNeighbors(faces[i]).first ==cell_index) ? 
				tess.GetFaceNeighbors(faces[i]).second : tess.GetFaceNeighbors(faces[i]).first;
			res[i] = &cells.at(other_cell);
		}
		return res;
	}

	void calc_naive_slope(ComputationalCell3D const& cell,
		Vector3D const& center, Vector3D const& cell_cm, double cell_volume, vector<ComputationalCell3D const*> const& neighbors,
		vector<Vector3D> const& neighbor_centers, vector<Vector3D> const& neigh_cm,Tessellation3D const& tess,
		Slope3D &res, Slope3D &temp,size_t /*index*/, vector<size_t> const& faces)
	{
		size_t n = neighbor_centers.size();
		if (n > 40)
		{
			UniversalError eo("Cell has too many neighbors in calc naive slope");
			eo.AddEntry("Cell x cor", center.x);
			eo.AddEntry("Cell y cor", center.y);
			eo.AddEntry("Cell z cor", center.z);
			throw eo;
		}
		// Create the matrix to invert and the vector to compare
		boost::array<double,9>  m;
		std::fill_n(m.begin(), 9, 0);
		for (size_t i = 0; i < n; ++i)
		{
			Vector3D c_ij = tess.FaceCM(faces[i]) - 0.5 * (neigh_cm[i] + cell_cm);
			const Vector3D r_ij = normalize(neighbor_centers[i] - center);
			const double A = tess.GetArea(faces[i]);
			m[0] -= c_ij.x*r_ij.x*A;
			m[1] -= c_ij.y*r_ij.x*A;
			m[2] -= c_ij.z*r_ij.x*A;
			m[3] -= c_ij.x*r_ij.y*A;
			m[4] -= c_ij.y*r_ij.y*A;
			m[5] -= c_ij.z*r_ij.y*A;
			m[6] -= c_ij.x*r_ij.z*A;
			m[7] -= c_ij.y*r_ij.z*A;
			m[8] -= c_ij.z*r_ij.z*A;
			if (i == 0)
			{
				ReplaceComputationalCell(temp.xderivative, *neighbors[i]);
				temp.xderivative *= r_ij.x*A;
				ReplaceComputationalCell(temp.yderivative, *neighbors[i]);
				temp.yderivative *= r_ij.y*A;
				ReplaceComputationalCell(temp.zderivative, *neighbors[i]);
				temp.zderivative *= r_ij.z*A;
			}
			else
			{
				ComputationalCellAddMult(temp.xderivative, *neighbors[i], r_ij.x*A);
				ComputationalCellAddMult(temp.yderivative, *neighbors[i], r_ij.y*A);
				ComputationalCellAddMult(temp.zderivative, *neighbors[i], r_ij.z*A);
			}
			ComputationalCellAddMult(temp.xderivative, cell, r_ij.x*A);
			ComputationalCellAddMult(temp.yderivative, cell, r_ij.y*A);
			ComputationalCellAddMult(temp.zderivative, cell, r_ij.z*A);
		}
		for (size_t i = 0; i < 9; ++i)
			m[i] /= cell_volume;
		m[0] += 1;
		m[4] += 1;
		m[8] += 1;
		// Find the det
		const double det = -m[2] * m[4] * m[6] + m[1] * m[5] * m[6] + m[2] * m[3] * m[7] - m[0] * m[5] * m[7] - 
			m[1] * m[3] * m[8] + m[0] * m[4] * m[8];
		// Check none singular
		if (std::abs(det) < 1e-10)
		{
			UniversalError eo("Singular matrix");
			eo.AddEntry("Cell x cor", center.x);
			eo.AddEntry("Cell y cor", center.y);
			eo.AddEntry("Cell z cor", center.z);
			eo.AddEntry("Cell volume", cell_volume);
			eo.AddEntry("Det was", det);
			throw eo;
		}
		// Invert the matrix
		boost::array<double, 9>  m_inv;
		std::fill_n(m_inv.begin(), 9, 0);
		m_inv[0] = m[4] * m[8] - m[5] * m[7];
		m_inv[1] = m[2] * m[7] - m[1] * m[8];
		m_inv[2] = m[1] * m[5] - m[2] * m[4];
		m_inv[3] = m[5] * m[6] - m[3] * m[8];
		m_inv[4] = m[0] * m[8] - m[2] * m[6];
		m_inv[5] = m[2] * m[3] - m[5] * m[0];
		m_inv[6] = m[3] * m[7] - m[6] * m[4];
		m_inv[7] = m[6] * m[1] - m[0] * m[7];
		m_inv[8] = m[4] * m[0] - m[1] * m[3];
		for (size_t i = 0; i < 9; ++i)
			m_inv[i] /= (2 * cell_volume * det);
		
		ReplaceComputationalCell(res.xderivative, temp.xderivative);
		res.xderivative *= m_inv[0];
		ComputationalCellAddMult(res.xderivative, temp.yderivative, m_inv[1]);
		ComputationalCellAddMult(res.xderivative, temp.zderivative, m_inv[2]);

		ReplaceComputationalCell(res.yderivative, temp.xderivative);
		res.yderivative *= m_inv[3];
		ComputationalCellAddMult(res.yderivative, temp.yderivative, m_inv[4]);
		ComputationalCellAddMult(res.yderivative, temp.zderivative, m_inv[5]);

		ReplaceComputationalCell(res.zderivative, temp.xderivative);
		res.zderivative *= m_inv[6];
		ComputationalCellAddMult(res.zderivative, temp.yderivative, m_inv[7]);
		ComputationalCellAddMult(res.zderivative, temp.zderivative, m_inv[8]);
	}


	double PressureRatio(ComputationalCell3D cell, vector<ComputationalCell3D const*> const& neigh)
	{
		double res = 1;
		double p = cell.pressure;
		for (size_t i = 0; i < neigh.size(); ++i)
		{
			if (p > neigh[i]->pressure)
				res = std::min(res, neigh[i]->pressure / p);
			else
				res = std::min(res, p / neigh[i]->pressure);
		}
		return res;
	}

	bool is_shock(Slope3D const& naive_slope, double cell_width, double shock_ratio,
		ComputationalCell3D const& cell, vector<ComputationalCell3D const*> const& neighbor_list, double pressure_ratio, double cs)
	{
		const bool cond1 = (naive_slope.xderivative.velocity.x + naive_slope.yderivative.velocity.y + naive_slope.zderivative.velocity.z)*
			cell_width < (-shock_ratio)*cs;
		const bool cond2 = PressureRatio(cell, neighbor_list) < pressure_ratio;
		return cond1 || cond2;
	}

	ComputationalCell3D interp(ComputationalCell3D const& cell, Slope3D const& slope,
		Vector3D const& target, Vector3D const& cm)
	{
		ComputationalCell3D res(cell);
		ComputationalCellAddMult(res, slope.xderivative, target.x - cm.x);
		ComputationalCellAddMult(res, slope.yderivative, target.y - cm.y);
		ComputationalCellAddMult(res, slope.zderivative, target.z - cm.z);
		return res;
	}

	void interp2(ComputationalCell3D &res, Slope3D const& slope,
		Vector3D const& target, Vector3D const& cm)
	{
		ComputationalCellAddMult(res, slope.xderivative, target.x - cm.x);
		ComputationalCellAddMult(res, slope.yderivative, target.y - cm.y);
		ComputationalCellAddMult(res, slope.zderivative, target.z - cm.z);
	}

	void slope_limit(ComputationalCell3D const& cell, Vector3D const& cm,
		vector<ComputationalCell3D const*> const& neighbors, Slope3D &slope,ComputationalCell3D &cmax,
		ComputationalCell3D &cmin,ComputationalCell3D &maxdiff,	ComputationalCell3D &mindiff,
		TracerStickerNames const& tracerstickernames,string const& skip_key,Tessellation3D const& tess,
		size_t /*cell_index*/, vector<size_t> const& faces)
	{
		ReplaceComputationalCell(cmax, cell);
		ReplaceComputationalCell(cmin, cell);
		// Find maximum.minimum neighbor values
		size_t nloop = neighbors.size();
		for (size_t i = 0; i < nloop; ++i)
		{
			ComputationalCell3D const& cell_temp = *neighbors[i];
			if (!skip_key.empty() && safe_retrieve(cell_temp.stickers, tracerstickernames.sticker_names, skip_key))
				continue;
			if (tess.BoundaryFace(faces[i]))
				continue;
			cmax.density = std::max(cmax.density, cell_temp.density);
			cmax.pressure = std::max(cmax.pressure, cell_temp.pressure);
			cmax.velocity.x = std::max(cmax.velocity.x, cell_temp.velocity.x);
			cmax.velocity.y = std::max(cmax.velocity.y, cell_temp.velocity.y);
			cmax.velocity.z = std::max(cmax.velocity.z, cell_temp.velocity.z);
			cmin.density = std::min(cmin.density, cell_temp.density);
			cmin.pressure = std::min(cmin.pressure, cell_temp.pressure);
			cmin.velocity.x = std::min(cmin.velocity.x, cell_temp.velocity.x);
			cmin.velocity.y = std::min(cmin.velocity.y, cell_temp.velocity.y);
			cmin.velocity.z = std::min(cmin.velocity.z, cell_temp.velocity.z);
			for (size_t j = 0; j < cell_temp.tracers.size(); ++j)
			{
				cmax.tracers[j] = std::max(cmax.tracers[j], cell_temp.tracers[j]);
				cmin.tracers[j] = std::min(cmin.tracers[j], cell_temp.tracers[j]);
			}
		}
		ReplaceComputationalCell(maxdiff, cmax);
		maxdiff -= cell;
		ReplaceComputationalCell(mindiff, cmin);
		mindiff -= cell;
		// limit the slope
		ComputationalCell3D centroid_val = interp(cell, slope, tess.FaceCM(faces[0]), cm);
		ComputationalCell3D dphi = centroid_val - cell;
		vector<double> psi(5 + cell.tracers.size(), 1);
		const size_t nedges = faces.size();
		for (size_t i = 0; i < nedges; ++i)
		{
			if (tess.BoundaryFace(faces[i]))
				continue;
			if (i > 0)
			{
				ReplaceComputationalCell(centroid_val, cell);
				interp2(centroid_val, slope, tess.FaceCM(faces[i]), cm);
				ReplaceComputationalCell(dphi, centroid_val);
				dphi -= cell;
			}
			// density
			if (std::abs(dphi.density) > 0.1*std::max(std::abs(maxdiff.density), std::abs(mindiff.density)) || centroid_val.density*cell.density < 0)
			{
				if (dphi.density > 1e-9*cell.density)
					psi[0] = std::min(psi[0], maxdiff.density / dphi.density);
				else
					if (dphi.density<-1e-9*cell.density)
						psi[0] = std::min(psi[0], mindiff.density / dphi.density);
			}
			// pressure
			if (std::abs(dphi.pressure) > 0.1*std::max(std::abs(maxdiff.pressure), std::abs(mindiff.pressure)) || centroid_val.pressure*cell.pressure < 0)
			{
				if (dphi.pressure > 1e-9*cell.pressure)
					psi[1] = std::min(psi[1], maxdiff.pressure / dphi.pressure);
				else
					if (dphi.pressure<-1e-9*cell.pressure)
						psi[1] = std::min(psi[1], mindiff.pressure / dphi.pressure);
			}
			// xvelocity
			if (std::abs(dphi.velocity.x) > 0.1*std::max(std::abs(maxdiff.velocity.x), std::abs(mindiff.velocity.x)) || centroid_val.velocity.x*cell.velocity.x < 0)
			{
				if (dphi.velocity.x > std::abs(1e-9*cell.velocity.x))
					psi[2] = std::min(psi[2], maxdiff.velocity.x / dphi.velocity.x);
				else
					if (dphi.velocity.x<-std::abs(1e-9*cell.velocity.x))
						psi[2] = std::min(psi[2], mindiff.velocity.x / dphi.velocity.x);
			}
			// yvelocity
			if (std::abs(dphi.velocity.y) > 0.1*std::max(std::abs(maxdiff.velocity.y), std::abs(mindiff.velocity.y)) || centroid_val.velocity.y*cell.velocity.y < 0)
			{
				if (dphi.velocity.y > std::abs(1e-9*cell.velocity.y))
					psi[3] = std::min(psi[3], maxdiff.velocity.y / dphi.velocity.y);
				else
					if (dphi.velocity.y < -std::abs(1e-9*cell.velocity.y))
						psi[3] = std::min(psi[3], mindiff.velocity.y / dphi.velocity.y);
			}
			// zvelocity
			if (std::abs(dphi.velocity.z) > 0.1*std::max(std::abs(maxdiff.velocity.z), std::abs(mindiff.velocity.z)) || centroid_val.velocity.z*cell.velocity.z < 0)
			{
				if (dphi.velocity.z > std::abs(1e-9*cell.velocity.z))
					psi[4] = std::min(psi[4], maxdiff.velocity.z / dphi.velocity.z);
				else
					if (dphi.velocity.z < -std::abs(1e-9*cell.velocity.z))
						psi[4] = std::min(psi[4], mindiff.velocity.z / dphi.velocity.z);
			}
			// tracers
			for (size_t j = 0; j < dphi.tracers.size(); ++j)
			{
				double cell_tracer = cell.tracers[j];
				double diff_tracer = maxdiff.tracers[j];
				if (std::abs(dphi.tracers[j]) > 0.1*std::max(std::abs(diff_tracer), std::abs(mindiff.tracers[j])) || (
					centroid_val.tracers[j] * cell_tracer < 0))
				{
					if (dphi.tracers[j] > std::abs(1e-9*cell_tracer))
						psi[5 + j] = std::min(psi[5 + j], diff_tracer / dphi.tracers[j]);
					else
						if (dphi.tracers[j] < -std::abs(1e-9 * cell_tracer))
							psi[5 + j] = std::min(psi[5 + j], mindiff.tracers[j] / dphi.tracers[j]);
				}
			}
		}
		slope.xderivative.density *= psi[0];
		slope.yderivative.density *= psi[0];
		slope.zderivative.density *= psi[0];
		slope.xderivative.pressure *= psi[1];
		slope.yderivative.pressure *= psi[1];
		slope.zderivative.pressure *= psi[1];
		slope.xderivative.velocity.x *= psi[2];
		slope.yderivative.velocity.x *= psi[2];
		slope.zderivative.velocity.x *= psi[2];
		slope.xderivative.velocity.y *= psi[3];
		slope.yderivative.velocity.y *= psi[3];
		slope.zderivative.velocity.y *= psi[3];
		slope.xderivative.velocity.z *= psi[4];
		slope.yderivative.velocity.z *= psi[4];
		slope.zderivative.velocity.z *= psi[4];
		size_t counter = 5;
		size_t N = slope.xderivative.tracers.size();
		for (size_t k = 0; k < N; ++k)
		{
			slope.xderivative.tracers[k] *= psi[counter];
			slope.yderivative.tracers[k] *= psi[counter];
			slope.zderivative.tracers[k] *= psi[counter];
			++counter;
		}
	}

	void shocked_slope_limit(ComputationalCell3D const& cell, Vector3D const& cm,
		vector<ComputationalCell3D const*> const& neighbors, 
		Slope3D  &slope, double diffusecoeff, TracerStickerNames const& tracerstickernames,
		string const& skip_key,Tessellation3D const& tess,size_t /*cell_index*/, vector<size_t> const& faces)
	{
		ComputationalCell3D cmax(cell), cmin(cell);
		size_t N = faces.size();
		// Find maximum values
		for (size_t i = 0; i < N; ++i)
		{
			ComputationalCell3D const& cell_temp = *neighbors[i];
			if (!skip_key.empty() && safe_retrieve(cell_temp.stickers, tracerstickernames.sticker_names, skip_key))
				continue;
			cmax.density = std::max(cmax.density, cell_temp.density);
			cmax.pressure = std::max(cmax.pressure, cell_temp.pressure);
			cmax.velocity.x = std::max(cmax.velocity.x, cell_temp.velocity.x);
			cmax.velocity.y = std::max(cmax.velocity.y, cell_temp.velocity.y);
			cmax.velocity.z = std::max(cmax.velocity.z, cell_temp.velocity.z);
			cmin.density = std::min(cmin.density, cell_temp.density);
			cmin.pressure = std::min(cmin.pressure, cell_temp.pressure);
			cmin.velocity.x = std::min(cmin.velocity.x, cell_temp.velocity.x);
			cmin.velocity.y = std::min(cmin.velocity.y, cell_temp.velocity.y);
			cmin.velocity.z = std::min(cmin.velocity.z, cell_temp.velocity.z);
			for (size_t j = 0; j < cell_temp.tracers.size(); ++j)
			{
				cmax.tracers[j] = std::max(cmax.tracers[j], cell_temp.tracers[j]);
				cmin.tracers[j] = std::min(cmin.tracers[j], cell_temp.tracers[j]);
			}
		}
		ComputationalCell3D maxdiff = cmax - cell, mindiff = cmin - cell;
		// limit the slope
		vector<double> psi(5 + cell.tracers.size(), 1);
		for (size_t i = 0; i<N; ++i)
		{
			if (!skip_key.empty() && safe_retrieve(neighbors[i]->stickers, tracerstickernames.sticker_names, skip_key))
				continue;
			ComputationalCell3D centroid_val = interp(cell, slope, tess.FaceCM(faces[i]), cm);
			ComputationalCell3D dphi = centroid_val - cell;
			// density
			if (std::abs(dphi.density) > 0.1*std::max(std::abs(maxdiff.density), std::abs(mindiff.density)) || centroid_val.density*cell.density < 0)
			{
				if (std::abs(dphi.density) > 1e-9*cell.density)
					psi[0] = std::min(psi[0], std::max(diffusecoeff*(neighbors[i]->density - cell.density) / dphi.density, 0.0));
			}
			// pressure
			if (std::abs(dphi.pressure) > 0.1*std::max(std::abs(maxdiff.pressure), std::abs(mindiff.pressure)) || centroid_val.pressure*cell.pressure < 0)
			{
				if (std::abs(dphi.pressure) > 1e-9*cell.pressure)
					psi[1] = std::min(psi[1], std::max(diffusecoeff*(neighbors[i]->pressure - cell.pressure) / dphi.pressure, 0.0));
			}
			// xvelocity
			if (std::abs(dphi.velocity.x) > 0.1*std::max(std::abs(maxdiff.velocity.x), std::abs(mindiff.velocity.x)) || centroid_val.velocity.x*cell.velocity.x < 0)
			{
				if (std::abs(dphi.velocity.x) > 1e-9*cell.velocity.x)
					psi[2] = std::min(psi[2], std::max(diffusecoeff*(neighbors[i]->velocity.x - cell.velocity.x) / dphi.velocity.x, 0.0));
			}
			// yvelocity
			if (std::abs(dphi.velocity.y) > 0.1*std::max(std::abs(maxdiff.velocity.y), std::abs(mindiff.velocity.y)) || centroid_val.velocity.y*cell.velocity.y < 0)
			{
				if (std::abs(dphi.velocity.y) > 1e-9*cell.velocity.y)
					psi[3] = std::min(psi[3], std::max(diffusecoeff*(neighbors[i]->velocity.y - cell.velocity.y) / dphi.velocity.y, 0.0));
			}
			// zvelocity
			if (std::abs(dphi.velocity.z) > 0.1*std::max(std::abs(maxdiff.velocity.z), std::abs(mindiff.velocity.z)) || centroid_val.velocity.z*cell.velocity.z < 0)
			{
				if (std::abs(dphi.velocity.y) > 1e-9*cell.velocity.y)
					psi[4] = std::min(psi[4], std::max(diffusecoeff*(neighbors[i]->velocity.z - cell.velocity.z) / dphi.velocity.z, 0.0));
			}
			// tracers
			size_t counter = 0;
			for (size_t j = 0; j < dphi.tracers.size(); ++j)
			{
				double cell_tracer = cell.tracers[j];
				double diff_tracer = maxdiff.tracers[j];
				double centroid_tracer = centroid_val.tracers[j];
				if (std::abs(dphi.tracers[j]) > 0.1*std::max(std::abs(diff_tracer), std::abs(mindiff.tracers[j])) ||
					centroid_tracer*cell_tracer < 0)
				{
					if (std::abs(dphi.tracers[j]) > std::abs(1e-9*cell_tracer))
						psi[5 + counter] = std::min(psi[5 + counter],
							std::max(diffusecoeff*(neighbors[i]->tracers[j] - cell_tracer) / dphi.tracers[j], 0.0));
				}
				++counter;
			}
		}
		slope.xderivative.density *= psi[0];
		slope.yderivative.density *= psi[0];
		slope.zderivative.density *= psi[0];
		slope.xderivative.pressure *= psi[1];
		slope.yderivative.pressure *= psi[1];
		slope.zderivative.pressure *= psi[1];
		slope.xderivative.velocity.x *= psi[2];
		slope.yderivative.velocity.x *= psi[2];
		slope.zderivative.velocity.x *= psi[2];
		slope.xderivative.velocity.y *= psi[3];
		slope.yderivative.velocity.y *= psi[3];
		slope.zderivative.velocity.y *= psi[3];
		slope.xderivative.velocity.z *= psi[4];
		slope.yderivative.velocity.z *= psi[4];
		slope.zderivative.velocity.z *= psi[4];
		size_t counter = 0;
		for (size_t k = 0; k < slope.xderivative.tracers.size(); ++k)
		{
			slope.xderivative.tracers[k] *= psi[5 + counter];
			slope.yderivative.tracers[k] *= psi[5 + counter];
			slope.zderivative.tracers[k] *= psi[5 + counter];
			++counter;
		}
	}

	void GetBoundarySlope(ComputationalCell3D const& cell, Vector3D const& cell_cm,
		vector<ComputationalCell3D const*> const& neighbors, vector<Vector3D> const& neigh_cm,
		Slope3D &res)
	{
		size_t Nneigh = neigh_cm.size();
		ComputationalCell3D PhiSy, PhiSx,PhiSz;
		PhiSy.tracers.resize(cell.tracers.size(), 0);
		PhiSx.tracers.resize(cell.tracers.size(), 0);
		PhiSz.tracers.resize(cell.tracers.size(), 0);
		double SxSy(0), Sy2(0), Sx2(0),SxSz(0),Sz2(0),SzSy(0);
		for (size_t i = 0; i < Nneigh; ++i)
		{
			PhiSy += (*neighbors[i] - cell)*(neigh_cm[i].y - cell_cm.y);
			PhiSx += (*neighbors[i] - cell)*(neigh_cm[i].x - cell_cm.x);
			PhiSz += (*neighbors[i] - cell)*(neigh_cm[i].z - cell_cm.z);
			SxSy += (neigh_cm[i].y - cell_cm.y)*(neigh_cm[i].x - cell_cm.x);
			Sx2 += (neigh_cm[i].x - cell_cm.x)*(neigh_cm[i].x - cell_cm.x);
			Sy2 += (neigh_cm[i].y - cell_cm.y)*(neigh_cm[i].y - cell_cm.y);
			SxSz += (neigh_cm[i].z - cell_cm.z)*(neigh_cm[i].x - cell_cm.x);
			SzSy += (neigh_cm[i].z - cell_cm.z)*(neigh_cm[i].y - cell_cm.y);
			Sz2 += (neigh_cm[i].z - cell_cm.z)*(neigh_cm[i].z - cell_cm.z);
		}
		double bottom = SxSz*SxSz*Sy2 + SxSy*SxSy*Sz2 - Sx2*Sy2*Sz2 - 2 * SxSy*SxSz*SzSy + Sx2*SzSy*Sz2;
		res.xderivative = (PhiSz*SxSz*Sy2 + PhiSy*SxSy*Sz2 - PhiSx*Sy2*Sz2 - PhiSz*SxSy*SzSy - PhiSy*SxSz*SzSy + 
			PhiSx*SzSy*SzSy)/bottom;
		res.yderivative = (PhiSz*SzSy*Sx2 + PhiSy*SxSz*SxSz - PhiSx*SxSz*SzSy - PhiSz*SxSy*SxSz - PhiSy*Sx2*Sz2 +
			PhiSx*SxSy*Sz2)/bottom;
		res.zderivative = (PhiSz*SxSy*SxSy - PhiSy*SxSy*SxSz - PhiSz*Sy2*Sx2 + PhiSx*SxSz*Sy2 + PhiSy*Sx2*SzSy -
			PhiSx*SxSy*SzSy)/bottom;
		res.xderivative.stickers = cell.stickers;
		res.yderivative.stickers = cell.stickers;
		res.zderivative.stickers = cell.stickers;
	}


	void calc_slope(Tessellation3D const& tess,vector<ComputationalCell3D> const& cells,size_t cell_index,bool slf,
		double shockratio,double diffusecoeff,double pressure_ratio,EquationOfState const& eos,
		const vector<string>& flat_tracers,Slope3D &naive_slope_,Slope3D & res,	Slope3D &temp1,ComputationalCell3D &temp2,
		ComputationalCell3D &temp3,ComputationalCell3D &temp4,ComputationalCell3D &temp5,vector<Vector3D> &neighbor_mesh_list,
		vector<Vector3D> &neighbor_cm_list,TracerStickerNames const& tracerstickernames,string const& skip_key)
	{
		vector<size_t> const& faces = tess.GetCellFaces(cell_index);
		GetNeighborMesh(tess, cell_index, neighbor_mesh_list,faces);
		GetNeighborCM(tess, cell_index, neighbor_cm_list,faces);
		vector<ComputationalCell3D const* > neighbor_list = GetNeighborCells(tess,cell_index, cells,faces);

		ComputationalCell3D const& cell = cells[cell_index];
		bool boundary_slope = false;
		size_t Nneigh = faces.size();
		for (size_t i = 0; i < Nneigh; ++i)
			if (tess.BoundaryFace(faces[i]))
			{
				boundary_slope = true;
				break;
			}
		if (boundary_slope)
			GetBoundarySlope(cell, tess.GetCellCM(cell_index),neighbor_list, neighbor_cm_list, res);
		else
			calc_naive_slope(cell, tess.GetMeshPoint(cell_index), tess.GetCellCM(cell_index),
				tess.GetVolume(cell_index), neighbor_list, neighbor_mesh_list, neighbor_cm_list,tess,res, temp1,
				cell_index,	faces);

		naive_slope_ = res;

		for (size_t i = 0; i < flat_tracers.size(); ++i)
		{
			size_t tindex = static_cast<size_t>(binary_find(tracerstickernames.tracer_names.begin(), tracerstickernames.tracer_names.end(),
				flat_tracers[i]) - tracerstickernames.tracer_names.begin());
			assert(tindex < tracerstickernames.tracer_names.size());
			res.xderivative.tracers[tindex] = 0;
			res.yderivative.tracers[tindex] = 0;
			res.zderivative.tracers[tindex] = 0;
		}

		if (slf)
		{
			if (!is_shock(res, tess.GetWidth(cell_index), shockratio, cell, neighbor_list, pressure_ratio,
				eos.dp2c(cell.density, cell.pressure, cell.tracers, tracerstickernames.tracer_names)))
			{
				slope_limit(cell, tess.GetCellCM(cell_index), neighbor_list, res, temp2, temp3,temp4,temp5,
					tracerstickernames,skip_key,tess,cell_index,faces);
			}
			else
			{
				shocked_slope_limit(cell, tess.GetCellCM(cell_index), neighbor_list,res, diffusecoeff, tracerstickernames,
					skip_key,tess,cell_index,faces);
			}
		}
	}

#ifdef RICH_MPI

		void exchange_ghost_slopes(Tessellation3D const& tess, vector<Slope3D> & slopes)
		{
			MPI_exchange_data(tess, slopes, true);
		}
#endif//RICH_MPI
}

void LinearGauss3D::Interp(ComputationalCell3D &res, ComputationalCell3D const& cell, size_t cell_index, Vector3D const& cm, Vector3D const& target)const
{
	res = interp(cell, rslopes_[cell_index], target, cm);
}

LinearGauss3D::LinearGauss3D(EquationOfState const& eos,Ghost3D const& ghost,bool slf,double delta_v,double theta,
	double delta_P,const vector<string>& flat_tracers,string skip_key) : eos_(eos), ghost_(ghost),rslopes_(),
	naive_rslopes_(),slf_(slf),shockratio_(delta_v),diffusecoeff_(theta),pressure_ratio_(delta_P),
	flat_tracers_(flat_tracers),skip_key_(skip_key),to_skip_() {}

void LinearGauss3D::operator()(const Tessellation3D& tess, const vector<ComputationalCell3D>& cells, double time,
	vector<pair<ComputationalCell3D, ComputationalCell3D> > &res, TracerStickerNames const& tracerstickersnames) const
{
	const size_t CellNumber = tess.GetPointNo();
	vector<size_t> boundaryedges;
	// Get ghost points
	boost::container::flat_map<size_t, ComputationalCell3D> ghost_cells = ghost_.operator()(tess, cells, time, tracerstickersnames);
	// Copy ghost data into new cells vector
	vector<ComputationalCell3D> new_cells(cells);
	new_cells.resize(tess.GetTotalPointNumber());
	for (boost::container::flat_map<size_t, ComputationalCell3D>::const_iterator it = ghost_cells.begin(); it !=
		ghost_cells.end(); ++it)
		new_cells[it->first] = it->second;
	// Prepare slopes
	rslopes_.resize(CellNumber, Slope3D(cells[0], cells[0],cells[0]));
	naive_rslopes_.resize(CellNumber);
	Slope3D temp1(cells[0], cells[0],cells[0]);
	ComputationalCell3D temp2(cells[0]);
	ComputationalCell3D temp3(cells[0]);
	ComputationalCell3D temp4(cells[0]);
	ComputationalCell3D temp5(cells[0]);
	vector<Vector3D> neighbor_mesh_list;
	vector<Vector3D> neighbor_cm_list;
	res.resize(tess.GetTotalFacesNumber(),pair<ComputationalCell3D,ComputationalCell3D>(cells[0],cells[0]));
	for (size_t i = 0; i < CellNumber; ++i)
	{
		calc_slope(tess, new_cells, i, slf_, shockratio_, diffusecoeff_, pressure_ratio_, eos_,
			flat_tracers_, naive_rslopes_[i], rslopes_[i], temp1, temp2, temp3, temp4, temp5,
			neighbor_mesh_list, neighbor_cm_list, tracerstickersnames, skip_key_);
		vector<size_t> const& faces = tess.GetCellFaces(i);
		const size_t nloop = faces.size();
		for (size_t j = 0; j < nloop; ++j)
		{
			if (tess.GetFaceNeighbors(faces[j]).first == i)
			{
				ReplaceComputationalCell(res[faces[j]].first,new_cells[i]);
				interp2(res[faces[j]].first,rslopes_[i], tess.FaceCM(faces[j]), tess.GetCellCM(i));
				CheckCell(res[faces[j]].first);
				if (tess.GetFaceNeighbors(faces[j]).second > CellNumber)
					boundaryedges.push_back(faces[j]);
			}
			else
			{
				ReplaceComputationalCell(res[faces[j]].second,new_cells[i]);
				interp2(res[faces[j]].second,rslopes_[i], tess.FaceCM(faces[j]), tess.GetCellCM(i));
				CheckCell(res[faces[j]].second);
				if (tess.GetFaceNeighbors(faces[j]).first > CellNumber)
					boundaryedges.push_back(faces[j]);
			}
		}
	}
#ifdef RICH_MPI
	// communicate ghost slopes
	exchange_ghost_slopes(tess, rslopes_);
#endif //RICH_MPI
	// Interpolate the boundary edges
	size_t Nboundary = boundaryedges.size();
	for (size_t i = 0; i < Nboundary; ++i)
	{
		size_t N0 = tess.GetFaceNeighbors(boundaryedges[i]).first;
		if(N0 > CellNumber)
		{
			ReplaceComputationalCell(res[boundaryedges[i]].first, new_cells[N0]);
#ifdef RICH_MPI
			if(tess.BoundaryFace(boundaryedges[i]))
				interp2(res[boundaryedges[i]].first, ghost_.GetGhostGradient(tess, cells, rslopes_, N0, time, boundaryedges[i],
					tracerstickersnames), tess.FaceCM(boundaryedges[i]), tess.GetCellCM(N0));
			else
				interp2(res[boundaryedges[i]].first,rslopes_[N0], tess.FaceCM(boundaryedges[i]), tess.GetCellCM(N0));
#else
			interp2(res[boundaryedges[i]].first, ghost_.GetGhostGradient(tess, cells, rslopes_, N0,time,boundaryedges[i],
				tracerstickersnames), tess.FaceCM(boundaryedges[i]),tess.GetCellCM(N0));
			CheckCell(res[boundaryedges[i]].first);
#endif //RICH_MPI
		}
		else
		{
			N0 = tess.GetFaceNeighbors(boundaryedges[i]).second;
			res[boundaryedges[i]].second = new_cells[N0];
#ifdef RICH_MPI
			if (tess.BoundaryFace(boundaryedges[i]))
				interp2(res[boundaryedges[i]].second, ghost_.GetGhostGradient(tess, cells, rslopes_, N0, time, boundaryedges[i],
					tracerstickersnames), tess.FaceCM(boundaryedges[i]), tess.GetCellCM(N0));
			else
				interp2(res[boundaryedges[i]].second, rslopes_[N0], tess.FaceCM(boundaryedges[i]), tess.GetCellCM(N0));
#else
			interp2(res[boundaryedges[i]].second, ghost_.GetGhostGradient(tess, cells, rslopes_, N0, time, boundaryedges[i],
				tracerstickersnames), tess.FaceCM(boundaryedges[i]),tess.GetCellCM(N0));
			CheckCell(res[boundaryedges[i]].second);
#endif //RICH_MPI
		}
	}
}


vector<Slope3D>& LinearGauss3D::GetSlopes(void)const
{
	return rslopes_;
}

vector<Slope3D>& LinearGauss3D::GetSlopesUnlimited(void)const
{
	return naive_rslopes_;
}

