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
#include "../basic/basic_types.h"

#ifdef HAS_GUROBI

#include <gurobi_c++.h>


bool LinearProgramSolver::_solve_GUROBI(const LinearProgram* program) {
	try {
		typedef Variable<double>			Variable;
		typedef LinearExpression<double>	Objective;
		typedef LinearConstraint<double>	Constraint;

		const std::vector<Variable>& variables = program->variables();
		if (variables.empty()) {
			std::cerr << "variable set is empty" << std::endl;
			return false;
		}

		static GRBEnv env = GRBEnv();
		env.set(GRB_IntParam_LogToConsole, 0);

		GRBModel model = GRBModel(env);

		// create variables
		std::vector<GRBVar> X(variables.size());
		for (std::size_t i = 0; i < variables.size(); ++i) {
			const Variable& var = variables[i];

			double lb, ub;
			var.get_double_bounds(lb, ub);

			char vtype = GRB_CONTINUOUS;
			if (var.variable_type() == Variable::INTEGER)
				vtype = GRB_INTEGER;
			else if (var.variable_type() == Variable::BINARY)
				vtype = GRB_BINARY;

			X[i] = model.addVar(lb, ub, 0.0, vtype);
		}

		// Integrate new variables
		model.update();

		// Set objective
		GRBLinExpr obj;

		const Objective& objective = program->objective();
		const std::unordered_map<std::size_t, double>& obj_coeffs = objective.coefficients();
		std::unordered_map<std::size_t, double>::const_iterator it = obj_coeffs.begin();
		for (; it != obj_coeffs.end(); ++it) {
			std::size_t var_idx = it->first;
			double coeff = it->second;
			obj += coeff * X[var_idx];
		}
		model.setObjective(obj, program->objective_sense() == LinearProgram::MINIMIZE ? GRB_MINIMIZE : GRB_MAXIMIZE);

		// Add constraints
		const std::vector<Constraint>& constraints = program->constraints();
		for (std::size_t i = 0; i < constraints.size(); ++i) {
			GRBLinExpr expr;
			const Constraint& cstr = constraints[i];
			const std::unordered_map<std::size_t, double>& cstr_coeffs = cstr.coefficients();
			std::unordered_map<std::size_t, double>::const_iterator cur = cstr_coeffs.begin();
			for (; cur != cstr_coeffs.end(); ++cur) {
				std::size_t var_idx = cur->first;
				double coeff = cur->second;
				expr += coeff * X[var_idx];
			}

			switch (cstr.bound_type())
			{
			case Constraint::FIXED:
				model.addConstr(expr == cstr.get_single_bound());
				break;
			case Constraint::LOWER:
				model.addConstr(expr >= cstr.get_single_bound());
				break;
			case Constraint::UPPER:
				model.addConstr(expr <= cstr.get_single_bound());
				break;
			case Constraint::DOUBLE: {
				double lb, ub;
				cstr.get_double_bounds(lb, ub);
				model.addConstr(expr >= lb);
				model.addConstr(expr <= ub);
				break;
				}
			default:
				break;
			}
		}

		// Optimize model
		model.optimize();

		int status = model.get(GRB_IntAttr_Status);
		switch (status) {
		case GRB_OPTIMAL: {
			objective_value_ = model.get(GRB_DoubleAttr_ObjVal);
			result_.resize(variables.size());
			for (std::size_t i = 0; i < variables.size(); ++i) {
				result_[i] = X[i].get(GRB_DoubleAttr_X);
			}
			break;
		}
		
		case GRB_INF_OR_UNBD:
			std::cerr << "model is infeasible or unbounded" << std::endl;
			break;

		case GRB_INFEASIBLE:
			std::cerr << "model is infeasible" << std::endl;
			break;

		case GRB_UNBOUNDED:
			std::cerr << "model is unbounded" << std::endl;
			break;

		default:
			std::cerr << "optimization was stopped with status = " << status << std::endl;
			break;
		}

		return (status == GRB_OPTIMAL);
	}
	catch (GRBException e) {
		std::cerr << "Error code = " << e.getErrorCode() << std::endl;
		std::cerr << e.getMessage() << std::endl;
	}
	catch (...) {
		std::cerr << "Exception during optimization" << std::endl;
	}

	return false;
}

#endif