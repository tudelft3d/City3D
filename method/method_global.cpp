#include "method_global.h"


namespace Method
{

	double lambda_data_fitting = 0.34;
	double lambda_model_height = 0.04;
	double lambda_model_complexity = 0.62;

    double number_region_growing=40;
    double point_density=0.15;

	double coincident_threshold = 1e-7;

	//LinearProgramSolver::SolverName solver_name = LinearProgramSolver::SCIP;
	LinearProgramSolver::SolverName solver_name = LinearProgramSolver::GUROBI;

	//________________ names for various quality measures ____________________

	std::string facet_attrib_supporting_vertex_group = "facet_supporting_vertex_group";
	std::string facet_attrib_supporting_point_num = "facet_supporting_point_num";
	std::string facet_attrib_facet_area = "facet_area";
	std::string facet_attrib_covered_area = "facet_covered_area";


}
