#ifndef _ALGORITHM_POINT_SET_REGIONGROWING_H_
#define _ALGORITHM_POINT_SET_REGIONGROWING_H_





#include "../model/vertex_group.h"
#include <string>
#include <vector>
#include <list>
#include <set>

class PointSet;

class Region_Growing_Dectetor
{
public:
	// for entire point cloud. Returns the extracted primitives.
	std::vector<VertexGroup::Ptr> detect(
		PointSet* pset,
		unsigned int min_support = 100	// the minimal number of points required for a primitive
	);

	// for a subset of the point cloud. Returns the extracted primitives.
		// for a subset of the point cloud. Returns the extracted primitives.
	std::vector<VertexGroup::Ptr> detect(
		PointSet* pset,
		const std::vector<unsigned int>& vertitces,
		unsigned int min_support = 1000	// the minimal number of points required for a primitive
	);

	std::vector<unsigned int> unassigned_points;
};


#endif