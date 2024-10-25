#pragma once



#include "../model/vertex_group.h"
#include "../model/map.h"
#include "../model/point_set.h"
#include "../math/linear_program_solver.h"


#include <vector>

class PolyFitInfo;

class Reconstruction
{
 public:
	Reconstruction() {}
	~Reconstruction() {}

    /// segment the scene point cloud using footprint (to obtain individual buildings)
	void segmentation(PointSet* pset, Map* foot_print);

    /// extract the roof planes for each building
	bool extract_roofs(PointSet* pset, Map* foot_print);

    /// generate the footprint for single building
    Map *generate_polygon(PointSet *pSet, double footprint_height=-5.97,double denisty=0.2);

	/// reconstruct mesh models of the buildings in the scene.
	/// Note: the reconstructed models will be merged into 'result'.
	bool reconstruct(PointSet* pset, Map* foot_print, Map* result, LinearProgramSolver::SolverName solver_name, bool update_display = false);

    /// user provided footprint data may contain dense polylines representing curved structures, which is necessary
    /// to be simplified before pairwise intersection.
    Map* simplify_footprint(Map* foot_print) const;

 private:

    std::vector<std::vector<int>> detect_height_jump(PointSet* pset,
                                                     Map::Facet* footprint,
                                                     double min_height,
                                                     double max_height);

    std::vector<std::vector<int>> compute_height_field(PointSet* pset, Map::Facet* footprint);

    PointSet* create_roof_point_set(const PointSet* pset,
		const std::vector<VertexGroup::Ptr>& segments, VertexGroup* building);

	PointSet* create_projected_point_set(const PointSet* pset, const PointSet* roof);

	int extract_building_roof(PointSet* pset,
		VertexGroup* building,
		unsigned int min_support = 40);

	Map* reconstruct_single_building(PointSet* roof_pset, const std::vector<vec3>& line_segments, Map::Facet* footprint,
                                     LinearProgramSolver::SolverName solver_name);

	std::vector<vec3> compute_line_segment(PointSet* seg_pset, PointSet* roof_pset, Map::Facet* footprint);

	void extrude_boundary_to_ground(Map* model, const Plane3d& ground, PolyFitInfo* polyfit_info);

	int width = 400, height = 400;

    std::string cloud_file_name_;

};

