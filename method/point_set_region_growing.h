#ifndef _ALGORITHM_POINT_SET_REGIONGROWING_H_
#define _ALGORITHM_POINT_SET_REGIONGROWING_H_




#include "method_common.h"
#include "../model/vertex_group.h"
#include <string>
#include <vector>
#include <list>
#include <set>

class PointSet;

class METHOD_API Region_Growing_Dectetor
{
public:
	// for entire point cloud. Returns the extracted primitives.
	std::vector<VertexGroup::Ptr> detect(
		PointSet* pset,
		unsigned int min_support = 100,	// the minimal number of points required for a primitive
		double dist_thresh = 0.005,	// relative to the bounding box width. NOTE: Internally the distance threshold is taken as 3 * distance_threshold!!!
		double bitmap_reso = 0.02,	// relative to the bounding box width. NOTE: This threshold is NOT multiplied internally!
		double normal_thresh = 0.8 // the cos of the maximal normal deviation
	);

	// for a subset of the point cloud. Returns the extracted primitives.
		// for a subset of the point cloud. Returns the extracted primitives.
	std::vector<VertexGroup::Ptr> detect(
		PointSet* pset,
		const std::vector<unsigned int>& vertitces,
		unsigned int min_support = 1000,	// the minimal number of points required for a primitive
		double dist_thresh = 0.005,	// relative to the bounding box width. NOTE: Internally the distance threshold is taken as 3 * distance_threshold!!!
		double bitmap_reso = 0.02,	// relative to the bounding box width. NOTE: This threshold is NOT multiplied internally!
		double normal_thresh = 0.8	// the cos of the maximal normal deviation

	);

	std::vector<unsigned int> unassigned_points;
};


#endif