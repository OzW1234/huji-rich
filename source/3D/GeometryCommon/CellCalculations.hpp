/* !
\file CellCalculations.hpp
\brief Calculations for Cells
\author Itay Zandbank
*/

#ifndef CELL_CALCULATIONS_HPP
#define CELL_CALCULATIONS_HPP

#include <vector>

#include "Face.hpp"
#include "Tetrahedron.hpp"

//!\brief Splits a cell into tetrahedra, all touching the center of the cell
//!\param cell The faces surrounding the cell
//!\param tetrahedra The result of the split - the tetrahedra cover the entire cell.
//!\remark We don't remember the result but rather use an output argument because this code is C++03 compliant with no move semantics - this is a performance issue.
void SplitCell(const std::vector<const Face *> &cell, std::vector<Tetrahedron> &tetrahedra);

//!\brief Calculates the volume and center-of-mass of a cell
//!\param cell The faces surrounding the cell
//!\param volume The volume of the cell
//!\param centerOfMass The cell's Center of Mass
void CalculateCellDimensions(const std::vector<const Face *> &cell, double &volume, Vector3D &centerOfMass);

#endif