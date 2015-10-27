/*! \file amr.hpp
\brief Abstract class for amr
\author Elad Steinberg
*/

#ifndef AMR_HPP
#define AMR_HPP 1

#include "computational_cell_2d.hpp"
#include "extensive.hpp"
#include "../common/equation_of_state.hpp"
#include "OuterBoundary.hpp"
#include "../../tessellation/tessellation.hpp"
#include "../../tessellation/ConvexHull.hpp"
#include "clipper.hpp"
#include "../test_2d/main_loop_2d.hpp"
#include "../../tessellation/polygon_overlap_area.hpp"
#include <boost/scoped_ptr.hpp>

//! \brief Abstract class for cell update scheme in amr
class AMRCellUpdater
{
public:

	/*! \brief Calculates the computational cell
	\param extensive Extensive conserved variables
	\param eos Equation of state
	\param volume Cell volume
	\param old_cell Old computational cell
	\return Computational cell
	*/
	virtual ComputationalCell ConvertExtensiveToPrimitve(const Extensive& extensive,const EquationOfState& eos,
		double volume,ComputationalCell const& old_cell) const = 0;

	//! \brief Class destructor
	virtual ~AMRCellUpdater(void);
};

// !\brief Abstract class for extensive update scheme in amr
class AMRExtensiveUpdater
{
public:

	/*! \brief Calculates the computational cell
	\param cell Computational cell
	\param eos Equation of state
	\param volume Cell volume
	\return Extensive
	*/
	virtual Extensive ConvertPrimitveToExtensive(const ComputationalCell& cell, const EquationOfState& eos,
		double volume) const = 0;

	//! \brief Class destructor
	virtual ~AMRExtensiveUpdater(void);
};

// !\brief Simple class for extensive update scheme in amr
class SimpleAMRExtensiveUpdater : public AMRExtensiveUpdater
{
public:
	Extensive ConvertPrimitveToExtensive(const ComputationalCell& cell, const EquationOfState& eos,
		double volume) const;
};

// !\brief Simple class for cell update scheme in amr
class SimpleAMRCellUpdater : public AMRCellUpdater
{
public:
	ComputationalCell ConvertExtensiveToPrimitve(const Extensive& extensive, const EquationOfState& eos,
		double volume, ComputationalCell const& old_cell) const;
};


class CellsToRemove
{
public:
	/*! 
	\brief Finds the cells to remove
	\param tess The tesselation
	\param cells The computational cells
	\param time The sim time
	\return The indeces of cells to remove with a corresponding merit which decides if there are neighboring cells which one to choose to remove
	*/
	virtual std::pair<vector<size_t>,vector<double> > ToRemove(Tessellation const& tess,
		vector<ComputationalCell> const& cells,double time)const=0;

	//! \brief Virtual destructor
	virtual ~CellsToRemove(void);
};

class CellsToRefine
{
public:
	/*!
	\brief Finds the cells to refine
		\param tess The tesselation
		\param cells The computational cells
		\param time The sim time
		\return The indeces of cells to remove
     */
	virtual vector<size_t> ToRefine(Tessellation const& tess, vector<ComputationalCell> const& cells, double time)const = 0;
	
	//! \brief Virtual destructor
	virtual ~CellsToRefine(void);
};


class AMR : public Manipulate
{
protected:
  void GetNewPoints
  (vector<size_t> const& ToRefine,
   Tessellation const& tess,
   vector<std::pair<size_t, Vector2D> > &NewPoints, 
   vector<Vector2D> &Moved,
   OuterBoundary const& obc)const;
public:
	/*!
	\brief Runs the AMR
	\param sim The sim object
	*/
	virtual void operator() (hdsim &sim) = 0;
	/*!
	\brief Runs the refine
	\param tess The tessellation
	\param cells The computational cells
	\param eos The equation of state
	\param extensives The extensive variables
	\param time The sim time
	*/
	virtual void UpdateCellsRefine(Tessellation &tess,
		OuterBoundary const& obc, vector<ComputationalCell> &cells, EquationOfState const& eos,
		vector<Extensive> &extensives, double time)const = 0;
	/*!
	\brief Runs the removal
	\param tess The tessellation
	\param cells The computational cells
	\param eos The equation of state
	\param extensives The extensive variables
	\param time The sim time
	\param obc The outer boundary conditions
	*/
	virtual void UpdateCellsRemove(Tessellation &tess,
		OuterBoundary const& obc, vector<ComputationalCell> &cells, vector<Extensive> &extensives,
		EquationOfState const& eos, double time)const = 0;
	//! \brief Virtual destructor
	virtual ~AMR(void);
};

//! \todo Make sure AMR works with all physical geometries
class ConservativeAMR : public AMR
{
private:
	CellsToRefine const& refine_;
	CellsToRemove const& remove_;
  SimpleAMRCellUpdater scu_;
  SimpleAMRExtensiveUpdater seu_;
	AMRCellUpdater* cu_;
	AMRExtensiveUpdater* eu_;

	vector<size_t> RemoveNeighbors(vector<double> const& merits, vector<size_t> const&
		candidates, Tessellation const& tess) const;

public:
	void operator() (hdsim &sim);

	ConservativeAMR
	(CellsToRefine const& refine,
	 CellsToRemove const& remove,
	 AMRCellUpdater* cu=0,
	 AMRExtensiveUpdater* eu=0);

	void UpdateCellsRefine(Tessellation &tess,
		OuterBoundary const& obc, vector<ComputationalCell> &cells,EquationOfState const& eos,
		vector<Extensive> &extensives,double time)const;

	void UpdateCellsRemove(Tessellation &tess,
		OuterBoundary const& obc, vector<ComputationalCell> &cells, vector<Extensive> &extensives,
		EquationOfState const& eos,double time)const;
};

class NonConservativeAMR : public AMR
{
private:
	CellsToRefine const& refine_;
	CellsToRemove const& remove_;
  SimpleAMRCellUpdater scu_;
  SimpleAMRExtensiveUpdater seu_;
	AMRExtensiveUpdater* eu_;

public:
	void operator() (hdsim &sim);

  NonConservativeAMR
  (CellsToRefine const& refine,
   CellsToRemove const& remove,
   AMRExtensiveUpdater* eu = 0);

	void UpdateCellsRefine(Tessellation &tess,
		OuterBoundary const& obc, vector<ComputationalCell> &cells, EquationOfState const& eos,
		vector<Extensive> &extensives, double time)const;

	void UpdateCellsRemove(Tessellation &tess,
		OuterBoundary const& obc, vector<ComputationalCell> &cells, vector<Extensive> &extensives,
		EquationOfState const& eos, double time)const;
};

#endif // AMR_HPP