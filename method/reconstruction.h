#pragma once

#include "method_common.h"

#include "../model/vertex_group.h"
#include "../model/map.h"
#include "../model/point_set.h"


#include <vector>

class PolyFitInfo;

class METHOD_API Reconstruction
{
 public:
	Reconstruction()
	{
	}
	~Reconstruction()
	{
	}

	std::vector<std::vector<int>> detect_height_jump(PointSet* pset,
		Map::Facet* footprint,
		double min_height,
		double max_height);

	std::vector<std::vector<int>> compute_height_field(PointSet* pset, Map::Facet* footprint);

	void segmentation(PointSet* pset, Map* foot_print);

	void extract_roofs(PointSet* pset, Map* foot_print);
	// the reconstructed models will be merged into 'result'
	bool reconstruct(PointSet* pset, Map* foot_print, Map* result, bool show);

 private:
	PointSet* create_roof_point_set(const PointSet* pset,
		const std::vector<VertexGroup::Ptr>& segments, VertexGroup* building);

	PointSet* create_projected_point_set(const PointSet* pset, const PointSet* roof);

	void extract_building_roof(PointSet* pset,
		VertexGroup* building,
		unsigned int min_support = 40);

	Map* reconstruct_single_building(PointSet* roof_pset, std::vector<vec3> line_segments, Map::Facet* footprint);

	std::vector<vec3> compute_line_segment(PointSet* seg_pset, PointSet* roof_pset, Map::Facet* footprint);

	void extrude_boundary_to_ground(Map* model, const Plane3d& ground, PolyFitInfo* polyfit_info);

	int width = 400, height = 400;


};

