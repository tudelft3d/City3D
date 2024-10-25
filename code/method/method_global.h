#ifndef _METHOD_GLOBAL_H_
#define _METHOD_GLOBAL_H_



#include "../math/linear_program_solver.h"

#include <string>


namespace Method {

	extern double lambda_data_fitting;
	extern double lambda_model_height;
	extern double lambda_model_complexity;

    extern double number_region_growing;
    extern double point_density;
    extern double ground_height;

	// - two points considered as coincident; 
	// - a point considered to be on a plane, etc.
	extern double coincident_threshold;

	//________________ names for various quality measures ____________________

	extern std::string facet_attrib_supporting_vertex_group;
	extern std::string facet_attrib_supporting_point_num;
	extern std::string facet_attrib_facet_area;
	extern std::string facet_attrib_covered_area;

    //________________ intermediate temp direction ____________________
    extern std::string intermediate_dir;
}


#endif