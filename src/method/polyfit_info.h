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

#ifndef _POLYFIT_DATA_H_
#define _POLYFIT_DATA_H_


#include "../math/math_types.h"
#include "../model/map_attributes.h"
#include "../model/vertex_group.h"

#include <vector>
#include <string>


class PointSet;
class Map;
class VertexGroup;


class PolyFitInfo {
public:
	PolyFitInfo() {}
	~PolyFitInfo() { clear(); }

	void generate(PointSet* pset, Map* mesh, std::vector < Plane3d* > &v,  bool use_conficence = false);

	bool ready_for_optimization(Map* mesh) const;

	void clear();

	std::vector<Plane3d*>  planes;		// including the bbox face planes
	double				   max_dist;	// maximum distance to the supporting plane

	MapFacetAttribute<VertexGroup*> facet_attrib_supporting_vertex_group;
	MapFacetAttribute<Plane3d*>		facet_attrib_supporting_plane;

	MapFacetAttribute<double>		facet_attrib_supporting_point_num;
	MapFacetAttribute<double>		facet_attrib_facet_area;
	MapFacetAttribute<double>		facet_attrib_covered_area;

	std::map<Map::Halfedge*, Plane3d*>			foot_print_edge_derived_plane;
	std::map<Map::Halfedge*, VertexGroup::Ptr>	foot_print_edge_derived_vertex_group;

private:
	// std::vector<unsigned int>& points returns the point indices projected in f.
	// returns the 'number' of points projected in f (accounts for a notion of confidence)
	double facet_points_projected_in(PointSet* pset, VertexGroup* g, MapTypes::Facet* f, double max_dist, std::vector<unsigned int>& points);

	// returns average spacing
	double compute_point_confidences(PointSet* pset, int s1 = 6, int s2 = 16, int s3 = 32);
};


#endif