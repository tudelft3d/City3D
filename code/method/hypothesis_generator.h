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

#ifndef _HYPOTHESIS_GENERATOR_
#define _HYPOTHESIS_GENERATOR_


#include "../math/polygon2d.h"
#include "../model/vertex_group.h"
#include "../model/map_attributes.h"
#include "../model/point_set.h"

#include <string>
#include <vector>


class Map;
class PointSet;
class VertexGroup;
class MapEditor;
class PolyFitInfo;

namespace MapTypes {
	class Vertex;
	class Facet;
	class Halfedge;
}


class HypothesisGenerator
{
 public:
	HypothesisGenerator(PointSet* roof_pset);
	~HypothesisGenerator();

	void refine_planes();

	Map* generate(PolyFitInfo* polyfit_info, Map::Facet* footprint, const std::vector<vec3>& line_segments);

	std::vector<Plane3d*> get_vertical_planes();

 private:
	void collect_valid_planes(PolyFitInfo* polyfit_info, Map::Facet* footprint, const std::vector<vec3>& line_segments);

	Map* compute_proxy_mesh(PolyFitInfo* polyfit_info, Map::Facet* footprint);

	// pairwise cut
	void pairwise_cut(Map* mesh);

 private:
	void bound(Map* mesh, const Map::Facet* footprint);
	void bound(Map* mesh, Map::Facet* f, const Map::Facet* footprint);
	MapFacetAttribute<Map*> inter_result;

	void merge(VertexGroup* g1, VertexGroup* g2, double max_dist);

	// test if face 'f' insects plane 'plane'
	bool do_intersect(MapTypes::Facet* f, MapTypes::Facet* plane);

	struct Intersection
	{
		enum Type
		{
			EXISTING_VERTEX, NEW_VERTEX
		};

		Intersection(Type t) : type(t), vtx(0), edge(0)
		{
		}
		Type type;

		// for EXISTING_VERTEX
		Map::Vertex* vtx;

		// for NEW_VERTEX
		MapTypes::Halfedge* edge;
		vec3				pos;
	};

	// compute the intersecting points of a face 'f' and a 'plane'. The intersecting points are returned 
	// by 'existing_vts' (if the plane intersects the face at its vertices) and 'new_vts' (if the plane 
	// intersects the face at its edges).
	void compute_intersections(
		MapTypes::Facet* f,
		MapTypes::Facet* plane,
		std::vector<Intersection>& intersections
		);

	std::vector<MapTypes::Facet*> cut(MapTypes::Facet* f, MapTypes::Facet* cutter, Map* mesh);

	// split an existing edge, meanwhile, assign the new edges the original source faces (the old edge 
	// lies in the intersection of the two faces)
	MapTypes::Vertex* split_edge(const Intersection& ep, MapEditor* editor, MapTypes::Facet* cutter);

	// collect all faces in 'mesh' that intersect 'face'
	std::set<MapTypes::Facet*> collect_intersecting_faces(MapTypes::Facet* face, Map* mesh,int i);

	void triplet_intersection(const std::vector<Plane3d*>& supporting_planes);

	// query the intersecting point for existing database, i.e., triplet_intersection_
	bool query_intersection(Plane3d* plane1, Plane3d* plane2, Plane3d* plane3, vec3& p);

	// compute the intersection of a plane triplet
	// returns true if the intersection exists (p returns the point)
	bool intersection_plane_triplet(const Plane3d* plane1, const Plane3d* plane2, const Plane3d* plane3, vec3& p);

 private:
	PointSet* roof_pset_;
	MapFacetAttribute<VertexGroup*> facet_attrib_supporting_vertex_group_;
	MapFacetAttribute<Plane3d*> facet_attrib_supporting_plane_;

	std::vector<VertexGroup::Ptr> planar_segments_;
	std::map<VertexGroup*, Plane3d*> vertex_group_plane_;
	std::vector<VertexGroup::Ptr> added_vertical_faces;
	VertexGroup::Ptr cut_horizontal_faces;
	VertexGroup::Ptr cut_horizontal_faces2;


	Plane3d* min_zplane;
	Plane3d* max_zplane;

	// How to use: triplet_intersection_[plane_min][plane_mid][plane_max]
	std::map<Plane3d*, std::map<Plane3d*, std::map<Plane3d*, vec3> > > triplet_intersection_;

	MapHalfedgeAttribute< std::set<Plane3d*> > edge_source_planes_;

	MapVertexAttribute< std::set<Plane3d*> > vertex_source_planes_;
};

#endif
