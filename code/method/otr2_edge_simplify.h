#ifndef LOD2_METHOD_OTR2_EDGE_SIMPLIFY_H_
#define LOD2_METHOD_OTR2_EDGE_SIMPLIFY_H_
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "Modified_Otr2.h"
#include <CGAL/squared_distance_2.h>
#include <CGAL/Surface_mesh.h>
#include <iostream>
#include <utility>
#include<vector>
#include "../model/map.h"
#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/create_offset_polygons_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_2 Point;
typedef K::Segment_2 Segment;
typedef K::Direction_2 Direction;
typedef K::Vector_2 Vector;
typedef K::Line_2 Line;
typedef CGAL::Optimal_transportation_reconstruction_2<K> Otr_2;
typedef CGAL::Polygon_2<K> Ply_2;
typedef CGAL::Straight_skeleton_2<K> Ss;

typedef boost::shared_ptr<Ply_2> PolygonPtr;
typedef boost::shared_ptr<Ss> SsPtr;

typedef std::vector<PolygonPtr> PolygonPtrVector;
class Otr2_edge_sim
{
 public:
	std::vector<vec3> edge_simplify(std::vector<std::vector<int>> edge_point, Map::Facet* footprint, double distance);

	FT max_distance(std::vector<Point> p, Otr_2& otr2);

    std::vector<vec3> cluster_lines(Otr_2& otr2, Map::Facet* footprint, int& width);


	Otr2_edge_sim(double x, double y, double res)
	{
		image_x_min = x;
		image_y_min = y;
		image_delta_res = res;
	};

	double image_x_min;
	double image_y_min;
	double image_delta_res;

};

#endif //LOD2_METHOD_OTR2_EDGE_SIMPLIFY_H_
