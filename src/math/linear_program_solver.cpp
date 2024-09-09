/*
Copyright (C) 2017  Liangliang Nan
https://3d.bk.tudelft.nl/liangliang/ - liangliang.nan@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include "linear_program_solver.h"


bool LinearProgramSolver::solve(const LinearProgram* program, SolverName solver /* = GUROBI */) {
	if (program->objective_sense() == LinearProgram::UNDEFINED) {
		std::cerr << "incomplete objective: undefined objective sense." << std::endl;
		return false;
	}

	result_.clear();

	switch (solver) {
#ifdef HAS_GUROBI
	case GUROBI:
		return _solve_GUROBI(program);
#endif
	case SCIP:
	default: // use SCIP
		return _solve_SCIP(program);
	}
}

