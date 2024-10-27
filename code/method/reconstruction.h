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
	Reconstruction();
	~Reconstruction() {}

    /// segment the scene point cloud using footprint (to obtain individual buildings)
	void segmentation(PointSet* pset, Map* footprint, bool simplify_footprint = true);

    /// extract the roof planes for each building
	bool extract_roofs(PointSet* pset, Map* footprint);

    /// generate the footprint for single building
    Map *generate_footprint(PointSet *pset);

	/// reconstruct mesh models of the buildings in the scene.
	/// Note: the reconstructed models will be merged into 'result'.
	bool reconstruct(PointSet* pset, Map* footprint, Map* result, LinearProgramSolver::SolverName solver_name, bool update_display = false);

 private:

    // user provided footprint data may contain dense polylines representing curved structures, which is necessary
    // to be simplified before pairwise intersection.
    void footprint_simplification(Map* footprint) const;

    std::vector<std::vector<int>> detect_height_jump(PointSet* pset,
                                                     Map::Facet* footprint,
                                                     double min_height,
                                                     double max_height);

    std::vector<std::vector<int>> compute_height_field(PointSet* pset, Map::Facet* footprint);

    PointSet* create_roof_point_set(const PointSet* pset,
		const std::vector<VertexGroup::Ptr>& segments, VertexGroup* building);

	PointSet* create_projected_point_set(const PointSet* pset, const PointSet* roof);

	int extract_building_roof(PointSet* pset, VertexGroup* building, unsigned int min_support = 40);

    // 'status' returns one of the following values:
    //      1: successful
    //      0: compromised (due to, e.g., too complex, detected inner wall excluded)
    //     -1: failed (due to, e.g., insufficient data and roofs not detected, solver timeout).
	Map* reconstruct_single_building(PointSet* roof_pset, const std::vector<vec3>& line_segments, Map::Facet* footprint,
                                     LinearProgramSolver::SolverName solver_name, const std::string& index_string, int& status);

	std::vector<vec3> compute_line_segment(PointSet* seg_pset, PointSet* roof_pset, Map::Facet* footprint);

	void extrude_boundary_to_ground(Map* model, const Plane3d& ground, PolyFitInfo* polyfit_info);

private:
    int width_;
    int height_;
};

