#include "condition_action_sequence_2.hpp"
#include "simple_flux_calculator.hpp"
#include "../../misc/utils.hpp"

ConditionActionSequence2::ConditionActionSequence2
(const vector<pair<const ConditionActionSequence::Condition*, const ConditionActionSequence::Action*> >& sequence,
	const vector<pair<const ConditionActionSequence::Condition*, const Action2*> >& sequence2,
	SpatialReconstruction const& interp):
	sequence_(sequence),sequence2_(sequence2),interp_(interp),edge_values_(vector<pair<ComputationalCell,
	ComputationalCell> >()){}

ConditionActionSequence2::~ConditionActionSequence2(void)
{}

namespace 
{
	void choose_action
		(const Edge& edge,
			const Tessellation& tess,
			const vector<ComputationalCell>& cells,
			const EquationOfState& eos,
			const Vector2D& edge_velocity,
			const vector<pair<const ConditionActionSequence::Condition*, const ConditionActionSequence::Action*> >& sequence,
		const vector<pair<const ConditionActionSequence::Condition*, const ConditionActionSequence2::Action2*> >& sequence2,
			pair<ComputationalCell,ComputationalCell> const& edge_values,Extensive &res)
	{
		for (size_t i = 0; i<sequence.size(); ++i) 
		{
			const pair<bool, bool> flag_aux = (*sequence[i].first)
				(edge, tess, cells);
			if (flag_aux.first)
			{
				(*sequence[i].second)(edge, tess, edge_velocity, cells, eos, flag_aux.second,res);
				return;
			}
		}
		for (size_t i = 0; i<sequence2.size(); ++i) 
		{
			const pair<bool, bool> flag_aux = (*sequence2[i].first)
				(edge, tess, cells);
			if (flag_aux.first)
			{
				(*sequence2[i].second)(edge, tess, edge_velocity, cells, eos, flag_aux.second, edge_values,res);
				return;
			}
		}
		throw UniversalError("Error in ConditionActionSequence");
	}
}

vector<Extensive> ConditionActionSequence2::operator()
(const Tessellation& tess,
	const vector<Vector2D>& edge_velocities,
	const vector<ComputationalCell>& cells,
	const vector<Extensive>& extensives,
	const CacheData& /*cd*/,
	const EquationOfState& eos,
	const double time,
	const double /*dt*/) const
{
	edge_values_.resize(static_cast<size_t>(tess.GetTotalSidesNumber()),
		pair<ComputationalCell, ComputationalCell>(cells[0], cells[0]));
	interp_.operator()(tess, cells, time,edge_values_);
	vector<Extensive> res(tess.getAllEdges().size(), extensives[0]);
	for (size_t i = 0; i<tess.getAllEdges().size(); ++i)
		choose_action
		(tess.getAllEdges()[i],
			tess,
			cells,
			eos,
			edge_velocities[i],
			sequence_,
			sequence2_,
			edge_values_[i],res[i]);
	return res;
}


ConditionActionSequence2::Action2::~Action2(void) {}

RegularFlux2::RegularFlux2(const RiemannSolver& rs) :
	rs_(rs) {}

namespace 
{
	void conserved_to_extensive
		(const Conserved& c, const ComputationalCell& cell,Extensive &res)
	{
		res.mass = c.Mass;
		res.momentum = c.Momentum;
		res.energy = c.Energy;
		boost::container::flat_map<string, double>::iterator it2 = res.tracers.begin();
		for (boost::container::flat_map<string, double>::const_iterator it =
			cell.tracers.begin();it != cell.tracers.end(); ++it,++it2)
			it2->second = (it->second)*c.Mass;
	}
}

void RegularFlux2::operator()
(const Edge& edge,
	const Tessellation& tess,
	const Vector2D& edge_velocity,
	const vector<ComputationalCell>& /*cells*/,
	const EquationOfState& eos,
	const bool /*aux*/,
	pair<ComputationalCell,ComputationalCell> const& edge_values,
	Extensive &res) const
{
	const Vector2D p = normalize
		(edge.vertices.second -
			edge.vertices.first);
	const Vector2D n = normalize
		(tess.GetMeshPoint(edge.neighbors.second) -
			tess.GetMeshPoint(edge.neighbors.first));
	const double v =
		ScalarProd(n, edge_velocity);
	const Conserved c = rotate_solve_rotate_back
		(rs_,
			convert_to_primitive
			(edge_values.first, eos),
			convert_to_primitive
			(edge_values.second, eos),
			v, n, p);
	conserved_to_extensive(c,c.Mass>0 ?	edge_values.first :	edge_values.second,res);
}

RigidWallFlux2::RigidWallFlux2
(const RiemannSolver& rs) :
	rs_(rs) {}

namespace 
{
	pair<Primitive, Primitive> rigid_wall_states
		(const Primitive& state,
			const Vector2D& p,
			const bool aux)
	{
		if (aux) {
			const Primitive left = state;
			const Primitive right = reflect(left, p);
			return pair<Primitive, Primitive>(left, right);
		}
		else {
			const Primitive right = state;
			const Primitive left = reflect(right, p);
			return pair<Primitive, Primitive>(left, right);
		}
	}
}

void RigidWallFlux2::operator()
(const Edge& edge,
	const Tessellation& tess,
	const Vector2D& /*edge_velocity*/,
	const vector<ComputationalCell>& /*cells*/,
	const EquationOfState& eos,
	const bool aux,
	pair<ComputationalCell,ComputationalCell> const& edge_values,
	Extensive &res) const
{
#ifndef RICH_MPI
	if (aux)
		assert(edge.neighbors.first >= 0 && edge.neighbors.first<tess.GetPointNo());
	else
		assert(edge.neighbors.second >= 0 && edge.neighbors.second<tess.GetPointNo());
#endif
	const Vector2D p = normalize
		(edge.vertices.second - edge.vertices.first);
	const Vector2D n =
		normalize
		(remove_parallel_component
			(aux ?
				edge.vertices.first - tess.GetMeshPoint(edge.neighbors.first) :
				tess.GetMeshPoint(edge.neighbors.second) - edge.vertices.first,
				p));
	const double v = 0;
	const pair<Primitive, Primitive> left_right =
		rigid_wall_states
		(convert_to_primitive
			(aux ? edge_values.first :edge_values.second,eos),
			p, aux);
	const Conserved c = rotate_solve_rotate_back
		(rs_,
			left_right.first,
			left_right.second,
			v, n, p);
	conserved_to_extensive(c,aux ? edge_values.first : edge_values.second,res);
}

Ratchet::Ratchet(const RiemannSolver& rs,bool in) :	
  in_(in),
  wall_(RigidWallFlux2(rs)),
  free_(FreeFlowFlux(rs)) {}


void Ratchet::operator()
(const Edge& edge,
	const Tessellation& tess,
	const Vector2D& edge_velocity,
	const vector<ComputationalCell>& cells,
	const EquationOfState& eos,
	const bool aux,
	const pair<ComputationalCell, ComputationalCell> & edge_values,
	Extensive &res) const
{
	Vector2D n = aux ? tess.GetMeshPoint(edge.neighbors.second) - tess.GetMeshPoint(edge.neighbors.first) :
		tess.GetMeshPoint(edge.neighbors.first) - tess.GetMeshPoint(edge.neighbors.second);
	if (ScalarProd(n, cells[static_cast<size_t>(aux ? edge.neighbors.first : edge.neighbors.second)].velocity)*(2*static_cast<double>(in_)-1) < 0)
		free_.operator()(edge,tess, edge_velocity, cells, eos, aux,res);
	else
		wall_.operator()(edge, tess, edge_velocity, cells, eos, aux,edge_values,res);
}
