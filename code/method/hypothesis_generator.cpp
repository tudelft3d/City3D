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

You should have received a copy of the GNU Gfeneral Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include "hypothesis_generator.h"
#include "cgal_types.h"
#include "alpha_shape.h"
#include "polyfit_info.h"
#include "method_global.h"
#include "../basic/progress.h"
#include "../basic/logger.h"
#include "../model/map_builder.h"
#include "../model/map_editor.h"
#include "../model/map_circulators.h"
#include "../model/map_geometry.h"
#include <CGAL/Projection_traits_xy_3.h>
#include <algorithm>

#define REMOVE_DEGENERATE_FACES  1

#if REMOVE_DEGENERATE_FACES

static void remove_degenerated_facets(Map* mesh)
{
	// You can't collect all the edges and then collapse them one by one, 
	// because collapsing one edge affects other neighboring edges.
	// std::vector<Map::Halfedge*> to_collapse;

	MapEditor editor(mesh);
	bool degenerated_facet_found = false;
	int count = 0;
	do // by checking edge length
    {
        degenerated_facet_found = false;
        FOR_EACH_EDGE(Map, mesh, it) {
            if (Geom::edge_length(it) < Method::coincident_threshold) {
                degenerated_facet_found = true;
                if (editor.collapse_edge(it)) {
                    ++count;
                    break;  // Liangliang: only one edge can be collapsed (one may affect others)
                }
            }
        }
    } while (degenerated_facet_found);

#if 1
    FOR_EACH_EDGE(Map, mesh, it) {
        const auto len = Geom::edge_length(it);
        if (len < Method::coincident_threshold)
            Logger::out("-") << count << "very short edge detected. length: " << len << std::endl;
    }
#endif

	if (count > 0)
		Logger::out("-") << count << " degenerate edges collapsed" << std::endl;
}

#endif

HypothesisGenerator::HypothesisGenerator(PointSet* pset)
	: pset_(pset)
{
}

HypothesisGenerator::~HypothesisGenerator()
{
}

static std::list<unsigned int> points_on_plane(VertexGroup* g, const Plane3d& plane, double dist_threshold)
{
	std::list<unsigned int> result;

	const std::vector<vec3>& points = g->point_set()->points();
	for (std::size_t i = 0; i < g->size(); ++i)
	{
		int idx = g->at(i);
		const vec3& p = points[idx];

		double sdist = plane.squared_ditance(p);
		double dist = std::sqrt(sdist);
		if (dist < dist_threshold)
			result.push_back(idx);
	}
	return result;
}

void HypothesisGenerator::merge(VertexGroup* g1, VertexGroup* g2, double max_dist)
{
	std::vector<VertexGroup::Ptr>& groups = pset_->groups();
	const std::vector<vec3>& points = pset_->points();

	std::vector<unsigned int> points_indices;
	points_indices.insert(points_indices.end(), g1->begin(), g1->end());
	points_indices.insert(points_indices.end(), g2->begin(), g2->end());

	VertexGroup::Ptr g = new VertexGroup;
	g->insert(g->end(), points_indices.begin(), points_indices.end());
	g->set_point_set(pset_);
	g->set_color(fused_color(g1->color(), static_cast<float>(g1->size()), g2->color(), static_cast<float>(g2->size())));
	pset_->fit_plane(g);
	groups.push_back(g);

	std::vector<VertexGroup::Ptr>::iterator pos = std::find(groups.begin(), groups.end(), g1);
	if (pos != groups.end())
	{
		groups.erase(pos);
	}
	else
	{
		std::cerr << "fatal error: vertex group doesn't exist" << std::endl;
	}

	pos = std::find(groups.begin(), groups.end(), g2);
	if (pos != groups.end())
	{
		groups.erase(pos);
	}
	else
	{
		std::cerr << "fatal error: vertex group doesn't exist" << std::endl;
	}
}

void HypothesisGenerator::refine_planes()
{
	std::vector<VertexGroup::Ptr>& groups = pset_->groups();
	if (groups.size() <= 1)
		return;

	//////////////////////////////////////////////////////////////////////////

	// remove near-vertical vertex groups

	const double vertical_threshold = 0.1f;

	std::vector<VertexGroup::Ptr> valid_groups;
	for (std::size_t i = 0; i < groups.size(); ++i)
	{
		VertexGroup* g = groups[i];
		const vec3& n = g->plane().normal();
		if (std::abs(n.z) > vertical_threshold)
			valid_groups.push_back(g);
	}
//	if (valid_groups.size() < groups.size())
//		Logger::out("-") << groups.size() - valid_groups.size() << " near-vertical segments discarded" << std::endl;
	groups = valid_groups;

	//////////////////////////////////////////////////////////////////////////

	// merge co-planar vertex groups
	int num = groups.size();
	const std::vector<vec3>& points = pset_->points();
	double avg_max_dist = 0;
	for (std::size_t i = 0; i < groups.size(); ++i)
	{
		VertexGroup* g = groups[i];
//		pset_->fit_plane(g);
		double g_max_dist = -FLT_MAX;
		const Plane3d& plane = g->plane();
		for (std::size_t j = 0; j < g->size(); ++j)
		{
			int idx = g->at(j);
			const vec3& p = points[idx];
			double sdist = plane.squared_ditance(p);
			g_max_dist = std::max(g_max_dist, std::sqrt(sdist));
		}

		avg_max_dist += g_max_dist;
	}
	avg_max_dist /= groups.size();
	avg_max_dist /= 2.0f;

	double theta = 10.0f;                // in degree
	theta = static_cast<double>(M_PI * theta / 180.0f);    // in radian
	bool merged = false;
	do
	{
		merged = false;
		std::sort(groups.begin(), groups.end(), VertexGroupCmpIncreasing());

		for (std::size_t i = 0; i < groups.size(); ++i)
		{
			VertexGroup* g1 = groups[i];
			const Plane3d& plane1 = g1->plane();
			const vec3& n1 = plane1.normal();
			double num_threshold = g1->size() / 5.0f;

			for (std::size_t j = i + 1; j < groups.size(); ++j)
			{
				VertexGroup* g2 = groups[j];
				const Plane3d& plane2 = g2->plane();
				double num_threshold1 = g2->size() / 5.0f;
				num_threshold = std::min(num_threshold, num_threshold1);

				const vec3& n2 = plane2.normal();
				if (std::abs(dot(n1, n2)) > std::cos(theta))
				{
					const std::list<unsigned int>& set1on2 = points_on_plane(g1, plane2, avg_max_dist);
					const std::list<unsigned int>& set2on1 = points_on_plane(g2, plane1, avg_max_dist);
					if (set1on2.size() > num_threshold && set2on1.size() > num_threshold)
					{
						merge(g1, g2, avg_max_dist);
						merged = true;
						break;
					}
				}
			}
			if (merged)
				break;
		}
	} while (merged);

	std::vector<VertexGroup::Ptr> valid_groups1;
	for (std::size_t i = 0; i < groups.size(); ++i)
	{
		VertexGroup* g = groups[i];
		const vec3& n = g->plane().normal();
		if (std::abs(n.z) > vertical_threshold)
			valid_groups1.push_back(g);
	}
	groups = valid_groups1;
	std::sort(groups.begin(), groups.end(), VertexGroupCmpDecreasing());

	if (num - groups.size() > 0)
	{
//		Logger::out("-") << num - groups.size() << " planar segments merged" << std::endl;
	}
}

void HypothesisGenerator::collect_valid_planes(PolyFitInfo* polyfit_info,
	Map::Facet* foot_print,
	const std::vector<vec3>& line_segments)
{
	std::vector<Plane3d*>& supporting_planes = polyfit_info->planes;

	planar_segments_.clear();
	added_vertical_faces.clear();
	vertex_group_plane_.clear();
	std::vector<VertexGroup::Ptr>& groups = pset_->groups();
	//////////////////////////////////
	//add vertical planes
	std::vector<vec3> temp_p;
	std::vector<vec3> temp_id;
	for (std::size_t i = 0; i < line_segments.size(); i++)
	{
		const vec3& p = line_segments[i];
		if (p[0] != 2)
		{
			temp_p.push_back(p);
		}
		else
		{
			temp_id.push_back(p);
		}

	}


	for (unsigned int id = 0; id < temp_id.size(); ++id)
	{
		vec3 v1 = temp_p[temp_id[id][1]];
		vec3 v2 = temp_p[temp_id[id][2]];
		vec3 v3 = v2;
		v3[2] = 5.;
		Plane3d plane(v1, v2, v3);
		PointSet::Ptr pset = new PointSet;
		VertexGroup::Ptr g = new VertexGroup;
		g->set_point_set(pset);
		pset->groups().push_back(g);
		g->set_plane(plane);
		added_vertical_faces.push_back(g);
	}


	for (std::size_t i = 0; i < added_vertical_faces.size(); ++i)
	{
		VertexGroup* g = added_vertical_faces[i];
		Plane3d* plane = new Plane3d(g->plane());
		supporting_planes.push_back(plane);
		vertex_group_plane_[g] = plane;
	}
	Logger::out("    -") << "num added vertical planes: " << added_vertical_faces.size() << std::endl;

	////////////////////////////////

	//add a cut plane
	PointSet::Ptr pset1 = new PointSet;
	VertexGroup::Ptr g1 = new VertexGroup;
	g1->set_point_set(pset1);
	pset1->groups().push_back(g1);
	auto bbz = pset_->bbox().z_max() + 0.5;
	z_plane = bbz;
	vec3 bbplane(0, 0, bbz);
	const vec3& n1 = vec3(0, 0, -1);
	Plane3d* plane1 = new Plane3d(bbplane, n1);
	g1->set_plane(*plane1);
	cut_horizontal_faces = g1;
	supporting_planes.push_back(plane1);
	vertex_group_plane_[g1] = plane1;

	//add a cut plane1. this code can be simpler, i leave it as later task
	PointSet::Ptr pset2 = new PointSet;
	VertexGroup::Ptr g2 = new VertexGroup;
	g2->set_point_set(pset2);
	pset2->groups().push_back(g2);
	auto bbz2 = pset_->bbox().z_min() - 0.5;
	z_bottom = bbz2;
	vec3 bbplane2(0, 0, bbz2);
	const vec3& n2 = vec3(0, 0, 1);
	Plane3d* plane2 = new Plane3d(bbplane2, n2);
	g2->set_plane(*plane2);
	cut_horizontal_faces2 = g2;
	supporting_planes.push_back(plane2);
	vertex_group_plane_[g2] = plane2;

	for (std::size_t i = 0; i < groups.size(); ++i)
	{
		VertexGroup* g = groups[i];
		Plane3d* plane = new Plane3d(g->plane());
		supporting_planes.push_back(plane);
		if (g->size())
			planar_segments_.push_back(g);
		vertex_group_plane_[g] = plane;

	}
	//////////////////////////////////////////////////////////////////////////

	std::map<Map::Halfedge*, VertexGroup::Ptr>
		& foot_print_edge_derived_vertex_group = polyfit_info->foot_print_edge_derived_vertex_group;
	std::map<Map::Halfedge*, Plane3d*>& foot_print_edge_derived_plane = polyfit_info->foot_print_edge_derived_plane;

	FacetHalfedgeCirculator cir(foot_print);
	for (; !cir->end(); ++cir)
	{
		PointSet::Ptr pset = new PointSet;
		VertexGroup::Ptr g = new VertexGroup;
		g->set_point_set(pset);
		pset->groups().push_back(g);

		Map::Halfedge* h = cir->halfedge();
		vec3 v1 = Geom::vector(h);
		v1 = normalize(v1);
		const vec3& v2 = vec3(0, 0, 1);
		const vec3& n = cross(v1, v2);
		Plane3d* plane = new Plane3d(h->vertex()->point(), n);
		g->set_plane(*plane);

		//planar_segments_.push_back(g);
		supporting_planes.push_back(plane);
		vertex_group_plane_[g] = plane;
		foot_print_edge_derived_plane[h] = plane;
		foot_print_edge_derived_plane[h->opposite()] = plane;
		foot_print_edge_derived_vertex_group[h] = g;
		foot_print_edge_derived_vertex_group[h->opposite()] = g;
	}

	Logger::out("    -") << "num initial planes:  " << polyfit_info->planes.size() << std::endl;
}

static void check_source_planes(Map* mesh)
{
	MapFacetAttribute<Plane3d*> facet_supporting_plane(mesh, "FacetSupportingPlane");
	MapHalfedgeAttribute<std::set<Plane3d*> > edge_source_planes(mesh, "EdgeSourcePlanes");
	MapVertexAttribute<std::set<Plane3d*> > vertex_source_planes(mesh, "VertexSourcePlanes");

	FOR_EACH_FACET(Map, mesh, it)
	{
		if (facet_supporting_plane[it] == nil)
			std::cerr << "fatal error: facet_supporting_plane[it] == nil " << std::endl;
	}

	FOR_EACH_HALFEDGE(Map, mesh, it)
	{
		const std::set<Plane3d*>& tmp = edge_source_planes[it];
		if (tmp.size() != 2)
			std::cerr << "fatal error: edge_source_planes[it].size() != 2. Size = " << tmp.size() << std::endl;
	}

	FOR_EACH_VERTEX(Map, mesh, it)
	{
		const std::set<Plane3d*>& tmp = vertex_source_planes[it];
		if (tmp.size() != 3)
			std::cerr << "vertex_source_planes[it].size() != 3. Size = " << tmp.size() << std::endl;
	}
}

Map* HypothesisGenerator::compute_proxy_mesh(PolyFitInfo* polyfit_info, Map::Facet* foot_print)
{

	Map* mesh = new Map;
	MapBuilder builder(mesh);

	MapFacetAttribute<VertexGroup*> facet_supporting_vertex_group(mesh, Method::facet_attrib_supporting_vertex_group);

	MapFacetAttribute<Plane3d*> facet_supporting_plane(mesh, "FacetSupportingPlane");
	MapHalfedgeAttribute<std::set<Plane3d*> > edge_source_planes(mesh, "EdgeSourcePlanes");
	MapVertexAttribute<std::set<Plane3d*> > vertex_source_planes(mesh, "VertexSourcePlanes");

	std::map<Map::Halfedge*, Plane3d*>& foot_print_edge_derived_plane = polyfit_info->foot_print_edge_derived_plane;
	std::map<Map::Halfedge*, VertexGroup::Ptr>
		& foot_print_edge_derived_vertex_group = polyfit_info->foot_print_edge_derived_vertex_group;
	std::vector<Plane3d*>& supporting_planes = polyfit_info->planes;
	builder.begin_surface();
	int idx = 0;
	std::vector<Map::Halfedge*> ring;
	FacetHalfedgeCirculator cir(foot_print);
	for (; !cir->end(); ++cir)
	{
		Map::Halfedge* h = cir->halfedge();
		ring.push_back(h);
	}
	for (std::size_t i = 0; i < planar_segments_.size(); ++i)
	{
		VertexGroup* g = planar_segments_[i];
		Plane3d* plane = vertex_group_plane_[g];
		assert(plane);
		builder.begin_facet();
#if 1    // reverse the orientation to make the entire model well oriented
		for (std::vector<Map::Halfedge*>::const_reverse_iterator it = ring.rbegin(); it != ring.rend(); ++it)
		{
#else
			for (std::vector<Map::Halfedge*>::const_iterator it = ring.begin(); it != ring.end(); ++it) {
#endif
			Map::Halfedge* h = *it;
			Map::Vertex* v = h->vertex();
			const vec3& s = v->point();
			const vec3& t = s + vec3(0, 0, 1);
			vec3 p;
			if (plane->intersection(Line3d::from_two_points(s, t), p))
			{
				builder.add_vertex(p);
				builder.add_vertex_to_facet(idx);
				++idx;

				Plane3d* plane1 = foot_print_edge_derived_plane[h];
				Plane3d* plane2 = foot_print_edge_derived_plane[h->next()];
				assert(plane1);
				assert(plane2);
				assert(plane1 != plane);
				assert(plane2 != plane);
				assert(plane2 != plane1);
				Map::Vertex* cur_v = builder.current_vertex();
				vertex_source_planes[cur_v].insert(plane);
				vertex_source_planes[cur_v].insert(plane1);
				vertex_source_planes[cur_v].insert(plane2);
			}
			else
			{
				Logger::err() << "vertical line should intersect roof" << std::endl;
				Logger::err() << plane->normal() << std::endl;

			}
		}
		builder.end_facet();
		facet_supporting_vertex_group[builder.current_facet()] = g;
		facet_supporting_plane[builder.current_facet()] = plane;
	}
	//
	Box3d m = mesh->bbox();
	auto height = m.z_max() - m.z_min();
	auto z_max_bb = m.z_max() + 1.1 * height;
	auto z_min_bb = m.z_min() - 1.1 * height;
    //two planes cut the concave boundary box
	const vec3& v2 = vec3(0, 0, z_min_bb);
	const vec3& nor = vec3(0, 0, -1);
	Plane3d* plane_ground = new Plane3d(v2, nor);
	min_zplane = plane_ground;
	const vec3& v3 = vec3(0, 0, z_max_bb);
	const vec3& nor1 = vec3(0, 0, 1);
	Plane3d* plane_top = new Plane3d(v3, nor1);
	max_zplane = plane_top;
	supporting_planes.push_back(plane_ground);
	supporting_planes.push_back(plane_top);

	PointSet::Ptr pset1 = new PointSet;
	VertexGroup::Ptr g1 = new VertexGroup;
	g1->set_point_set(pset1);
	pset1->groups().push_back(g1);
	g1->set_plane(*plane_top);
	vertex_group_plane_[g1] = plane_top;

	PointSet::Ptr pset2 = new PointSet;
	VertexGroup::Ptr g2 = new VertexGroup;
	g2->set_point_set(pset2);
	pset1->groups().push_back(g2);
	g2->set_plane(*plane_ground);
	vertex_group_plane_[g2] = plane_ground;

	for (std::size_t i = 0; i < added_vertical_faces.size(); ++i)
	{
		VertexGroup* g = added_vertical_faces[i];
		Plane3d* plane = vertex_group_plane_[g];

		assert(plane);

		std::vector<vec3> inter_points;
		std::vector<Plane3d*> inter_derived_plane;

		for (std::vector<Map::Halfedge*>::const_iterator it = ring.begin(); it != ring.end(); ++it)
		{
			Map::Halfedge* h = *it;
			Map::Vertex* v = h->vertex();
			const vec3& s = v->point();
			Map::Halfedge* h_next = h->next();
			Map::Vertex* vt = h_next->vertex();
			const vec3& t = vt->point();
			vec3 p;
			if (plane->intersection(s, t, p))
			{
				inter_points.push_back(p);
				inter_derived_plane.push_back(foot_print_edge_derived_plane[h->next()]);
			}
		}
		// compute a direction of the intersecting line of the two planes
		if (inter_points.size() > 1)
		{
			vec3 dir = inter_points[0] - inter_points[1];
			dir = normalize(dir);

			std::vector<unsigned int> indices;
			Geom::sort_along_direction(inter_points, dir, indices);

			// now we determine the line segment in each interval falls inside/outside the footprint.
			// This is done in 2D (by projection onto the face)

			Polygon2d plg; // the face polygon in 2D
			FacetHalfedgeConstCirculator cir1(foot_print);
			for (; !cir1->end(); ++cir1)
			{
				const vec3& p = cir1->halfedge()->vertex()->point();
				vec2 p1(p[0], p[1]);
				const vec2& q = p1;
				plg.push_back(q);
			}
			for (std::size_t i = 1; i < indices.size(); ++i)
			{
				int id0 = indices[i - 1];
				int id1 = indices[i];
				Plane3d* pt0_derived_plane = inter_derived_plane[id0];
				Plane3d* pt1_derived_plane = inter_derived_plane[id1];

				vec3& pt0 = inter_points[id0];
				vec3& pt1 = inter_points[id1];
				vec3 m = 0.5 * (pt0 + pt1);
				vec2 m1(m[0], m[1]);
				const vec2& mid = m1;
				if (Geom::point_is_in_polygon(plg, mid))
				{ // we may need cut at the two intersecting points
					vec3 p0 = pt0;
					p0.z = z_min_bb;
					vec3 p1 = p0;
					p1.z = z_max_bb;
					Plane3d* plane1 = pt0_derived_plane;
					vec3 p3 = pt1;
					p3.z = z_min_bb;
					vec3 p2 = p3;
					p2.z = z_max_bb;
					Plane3d* plane2 = pt1_derived_plane;
					assert(plane1);
					assert(plane2);
					assert(plane1 != plane);
					assert(plane2 != plane);
					assert(plane2 != plane1);
					builder.begin_facet();
					builder.add_vertex(p0);
					builder.add_vertex_to_facet(idx);
					++idx;
					Map::Vertex* cur_v = builder.current_vertex();
					vertex_source_planes[cur_v].insert(plane);
					vertex_source_planes[cur_v].insert(plane1);
					vertex_source_planes[cur_v].insert(min_zplane);

					builder.add_vertex(p1);
					builder.add_vertex_to_facet(idx);
					++idx;
					Map::Vertex* cur_v1 = builder.current_vertex();
					vertex_source_planes[cur_v1].insert(plane);
					vertex_source_planes[cur_v1].insert(plane1);
					vertex_source_planes[cur_v1].insert(max_zplane);

					builder.add_vertex(p2);
					builder.add_vertex_to_facet(idx);
					++idx;
					Map::Vertex* cur_v2 = builder.current_vertex();
					vertex_source_planes[cur_v2].insert(plane);
					vertex_source_planes[cur_v2].insert(plane2);
					vertex_source_planes[cur_v2].insert(max_zplane);

					builder.add_vertex(p3);
					builder.add_vertex_to_facet(idx);
					++idx;

					Map::Vertex* cur_v3 = builder.current_vertex();
					vertex_source_planes[cur_v3].insert(plane);
					vertex_source_planes[cur_v3].insert(plane2);
					vertex_source_planes[cur_v3].insert(min_zplane);
					builder.end_facet();

					facet_supporting_plane[builder.current_facet()] = plane;
					facet_supporting_vertex_group[builder.current_facet()] = g;

				}
			}

		}
	}
	builder.end_surface();

	FOR_EACH_HALFEDGE(Map, mesh, it)
	{
		Map::Halfedge* h = it;
		const std::set<Plane3d*>& set1 = vertex_source_planes[h->prev()->vertex()];
		const std::set<Plane3d*>& set2 = vertex_source_planes[h->vertex()];
		std::set<Plane3d*> faces;
		std::set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), std::inserter(faces, faces.begin()));
		assert(faces.size() == 2);
		edge_source_planes[h] = faces;
	}

	check_source_planes(mesh);

	return mesh;
}

// test if face f and plane intersect
bool HypothesisGenerator::do_intersect(MapTypes::Facet* f, MapTypes::Facet* plane)
{
	std::vector<Intersection> vts;
	compute_intersections(f, plane, vts);
	return (vts.size() > 1);
}
std::set<Map::Facet*> HypothesisGenerator::collect_intersecting_faces(Map::Facet* face, Map* mesh, int i)
{
	std::set<Map::Facet*> intersecting_faces;
	int count_insert = 0;
	FOR_EACH_FACET(Map, mesh, it)
	{
		Map::Facet* f = it;
		if (f != face &&
			facet_attrib_supporting_plane_[f] != facet_attrib_supporting_plane_[face] &&
			facet_attrib_supporting_vertex_group_[f] != facet_attrib_supporting_vertex_group_[face]
			)
		{

			if (do_intersect(f, face))
			{
				intersecting_faces.insert(f);
				count_insert++;
			}
		}
	}
	return intersecting_faces;
}

void sort(Plane3d*& plane0, Plane3d*& plane1, Plane3d*& plane2)
{
	std::vector<Plane3d*> planes({ plane0, plane1, plane2 });
	std::sort(planes.begin(), planes.end());
	plane0 = planes[0];
	plane1 = planes[1];
	plane2 = planes[2];
}

void HypothesisGenerator::compute_intersections(
	MapTypes::Facet* f,
	MapTypes::Facet* cutter,
	std::vector<Intersection>& vts)
{
	vts.clear();
	double on_plane_threshold = Method::coincident_threshold * Method::coincident_threshold;
	Plane3d* plane = facet_attrib_supporting_plane_[cutter];

	Map::Halfedge* h = f->halfedge();
	do
	{
		const vec3& s = h->opposite()->vertex()->point();
		const vec3& t = h->vertex()->point();
		if (plane->squared_ditance(t) <= on_plane_threshold)
		{        // plane cuts at vertex 't'
			Intersection it(Intersection::EXISTING_VERTEX);
			it.vtx = h->vertex();
			vts.push_back(it);
		}
		else if (plane->squared_ditance(s) > on_plane_threshold)
		{    // cut at the edge
			const std::set<Plane3d*>& source_planes = edge_source_planes_[h];
			if (source_planes.size() == 2)
			{  // if the edge was computed from two faces, I use the source faces for computing the intersecting point
				if (plane->intersection(s, t))
				{
					Plane3d* plane1 = *(source_planes.begin());
					Plane3d* plane2 = *(source_planes.rbegin());
					Plane3d* plane3 = plane;
					sort(plane1, plane2, plane3);

					if (plane3 != plane1 && plane3 != plane2)
					{
						vec3 p;
						if (query_intersection(plane1, plane2, plane3, p))
						{
							Intersection it(Intersection::NEW_VERTEX);
							it.edge = h;
							it.pos = p;
							vts.push_back(it);
						}
						else
						{
							if (intersection_plane_triplet(plane1, plane2, plane3, p))
							{
								triplet_intersection_[plane1][plane2][plane3] =
									p; // store the intersection in our data base
								Intersection it(Intersection::NEW_VERTEX);
								it.edge = h;
								it.pos = p;
								vts.push_back(it);
							}
							else
								Logger::warn("-") << "fatal error. should have intersection. " << std::endl;
						}
					}
					else
					{
						Logger::warn("-") << "fatal error. should have 3 different planes. " << std::endl;
					}
				}
			}
			else
			{
				vec3 p;
				if (plane->intersection(s, t, p))
				{
					Intersection it(Intersection::NEW_VERTEX);
					it.edge = h;
					it.pos = p;
					vts.push_back(it);
				}
			}
		}
		h = h->next();
	} while (h != f->halfedge());
}

// split an existing edge, meanwhile, assign the new edges the original source faces (the old edge lies on the intersection of the two faces)
MapTypes::Vertex* HypothesisGenerator::split_edge(const Intersection& ep, MapEditor* editor, MapTypes::Facet* cutter)
{
	const std::set<Plane3d*>& sfs = edge_source_planes_[ep.edge];

	MapTypes::Vertex* v = editor->split_edge(ep.edge);
	v->set_point(ep.pos);

	if (sfs.size() == 2)
	{
		MapTypes::Halfedge* h = v->halfedge();
		edge_source_planes_[h] = sfs;
		edge_source_planes_[h->opposite()] = sfs;

		h = h->next();
		edge_source_planes_[h] = sfs;
		edge_source_planes_[h->opposite()] = sfs;
	}
	else
	{
		Logger::warn() << "edge_source_planes.size != 2" << std::endl;
	}

	vertex_source_planes_[v] = sfs;
	vertex_source_planes_[v].insert(facet_attrib_supporting_plane_[cutter]);

	return v;
}

bool vertex_on_facet(Map::Vertex* v, Map::Facet* f)
{
	FacetHalfedgeConstCirculator cir(f);
	for (; !cir->end(); ++cir)
	{
		if (cir->halfedge()->vertex() == v)
			return true;
	}
	return false;
}

static bool halfedge_exists_between_vertices(Map::Vertex* v1, Map::Vertex* v2)
{
	Map::Halfedge* cir = v1->halfedge();
	do
	{
		if (cir->opposite()->vertex() == v2)
		{
			return true;
		}
		cir = cir->next_around_vertex();
	} while (cir != v1->halfedge());
	return false;
}

std::vector<Map::Facet*> HypothesisGenerator::cut(MapTypes::Facet* f, MapTypes::Facet* cutter, Map* mesh)
{
	// For concave faces, compute all the intersecting points, and then sort them along the intersecting line direction.
	// After that, check each interval (consequent intersecting points), creat an edge between the intersecting point
	// if the middle point lies inside the face,

	std::vector<Map::Facet*> pieces;

	std::vector<Intersection> vts;
	compute_intersections(f, cutter, vts);
	if (vts.size() < 2)
	{ // no actual intersection
		return pieces;
	}

	// compute a direction of the intersecting line of the two planes
	const Plane3d* plane = facet_attrib_supporting_plane_[f];
	const vec3& n1 = plane->normal();
	const vec3& n2 = facet_attrib_supporting_plane_[cutter]->normal();
	vec3 dir = cross(n1, n2);
	dir = normalize(dir);

	std::vector<vec3> intersecting_points;
	for (std::size_t i = 0; i < vts.size(); ++i)
	{
		const Intersection& it = vts[i];
		if (it.type == Intersection::EXISTING_VERTEX)
			intersecting_points.push_back(it.vtx->point());
		else
			intersecting_points.push_back(it.pos);
	}
	std::vector<unsigned int> indices;
	Geom::sort_along_direction(intersecting_points, dir, indices);

	// now we determine the line segment in each interval falls inside/outside the face.
	// This is done in 2D (by projection onto the face)

	VertexGroup* g = facet_attrib_supporting_vertex_group_[f];
	MapEditor editor(mesh);

	Polygon2d plg; // the face polygon in 2D
	FacetHalfedgeConstCirculator cir(f);
	for (; !cir->end(); ++cir)
	{
		const vec3& p = cir->halfedge()->vertex()->point();
		const vec2& q = plane->to_2d(p);
		plg.push_back(q);
	}
	for (std::size_t i = 1; i < indices.size(); ++i)
	{
		int id0 = indices[i - 1];
		int id1 = indices[i];
		const vec3& p0 = intersecting_points[id0];
		const vec3& p1 = intersecting_points[id1];
		const vec3& m = 0.5 * (p0 + p1);
		const vec2& mid = plane->to_2d(m);
		if (Geom::point_is_in_polygon(plg, mid))
		{ // we may need cut at the two intersecting points
			const Intersection& it0 = vts[id0];
			const Intersection& it1 = vts[id1];
			if (it0.type == Intersection::EXISTING_VERTEX && it1.type == Intersection::EXISTING_VERTEX
				&& halfedge_exists_between_vertices(it0.vtx, it1.vtx)) // cut at existing edge
				continue;

			Map::Vertex* v0 = 0;
			Map::Vertex* v1 = 0;
			if (it0.type == Intersection::NEW_VERTEX && it1.type == Intersection::NEW_VERTEX)
			{
				v0 = split_edge(it0, &editor, cutter);
				v1 = split_edge(it1, &editor, cutter);
			}
			else if (it0.type == Intersection::EXISTING_VERTEX && it1.type == Intersection::EXISTING_VERTEX)
			{
				v0 = it0.vtx;
				v1 = it1.vtx;
			}
			else
			{ // i.e., it0.type != it1.type, meaning cutting at an existing vertex and a edge
				if (it0.type == Intersection::EXISTING_VERTEX)
				{
					v0 = it0.vtx;
					v1 = split_edge(it1, &editor, cutter);
				}
				else
				{
					v0 = split_edge(it0, &editor, cutter);
					v1 = it1.vtx;
				}
			}

			// In general cases, the face might be cut into multiple pieces, I need to find the one on which
			// both v0 and v1 reside.
			Map::Facet* face = 0;
			if (vertex_on_facet(v0, f) && vertex_on_facet(v1, f))
				face = f;
			else
			{
				for (std::size_t j = 0; j < pieces.size(); ++j)
				{
					if (vertex_on_facet(v0, pieces[j]) && vertex_on_facet(v1, pieces[j]))
					{
						face = pieces[j];
						break;
					}
				}
			}
			if (!face)
			{
				Logger::err("-") << "the two vertices should lie on the same facet" << std::endl;
				continue;
			}

			Map::Halfedge* h0 = v0->halfedge();
			if (h0->facet() != face)
			{
				do
				{
					h0 = h0->next()->opposite();
					if (h0->facet() == face)
						break;
				} while (h0 != v0->halfedge());
			}
			assert(h0->facet() == face);

			Map::Halfedge* h1 = v1->halfedge();
			if (h1->facet() != face)
			{
				do
				{
					h1 = h1->next()->opposite();
					if (h1->facet() == face)
						break;
				} while (h1 != v1->halfedge());
			}
			assert(h1->facet() == face);

			if (editor.can_split_facet(h0, h1))
			{
				Map::Halfedge* h = editor.split_facet(h0, h1);
				if (h)
				{
					edge_source_planes_[h].insert(facet_attrib_supporting_plane_[f]);
					edge_source_planes_[h].insert(facet_attrib_supporting_plane_[cutter]);
					edge_source_planes_[h->opposite()].insert(facet_attrib_supporting_plane_[f]);
					edge_source_planes_[h->opposite()].insert(facet_attrib_supporting_plane_[cutter]);

					Map::Facet* f1 = h->facet();
					facet_attrib_supporting_vertex_group_[f1] = g;
					pieces.push_back(f1);
					Map::Facet* f2 = h->opposite()->facet();
					facet_attrib_supporting_vertex_group_[f2] = g;
					pieces.push_back(f2);
				}
				else
				{
					Logger::warn("-") << "fatal error. splitting facet failed. " << std::endl;
				}
			}
			else
			{
				Logger::warn("-") << "fatal error. cannot split facet. " << std::endl;
			}
		}
	}

	return pieces;
}

std::vector<Plane3d*> HypothesisGenerator::get_vertical_planes()
{
	std::vector<Plane3d*> m;
	for (std::size_t i = 0; i < added_vertical_faces.size(); i++)
	{
		VertexGroup* g = added_vertical_faces[i];
		Plane3d* p = vertex_group_plane_[g];
		m.push_back(p);
	}
	return m;
}

void HypothesisGenerator::pairwise_cut(Map* mesh)
{
	check_source_planes(mesh);
	int count_cut = 0;

	std::vector<MapTypes::Facet*> all_faces;
	FOR_EACH_FACET(Map, mesh, it)
	{
		MapTypes::Facet* f = it;
		all_faces.push_back(f);
	}

	for (std::size_t i = 0; i < all_faces.size(); ++i)
	{
		MapTypes::Facet* f = all_faces[i];
		std::set<MapTypes::Facet*> intersecting_faces = collect_intersecting_faces(f, mesh, i);
		if (intersecting_faces.empty())
			continue;
		std::vector<MapTypes::Facet*> cutting_faces;
		cutting_faces.insert(cutting_faces.end(), intersecting_faces.begin(), intersecting_faces.end());

		// 1. f will be cut by all the intersecting_faces
		//    note: after each cut, the original face doesn't exist any more and it is replaced by multiple pieces.
		//          then each piece will be cut by another face.

		std::vector<MapTypes::Facet*> pieces;
		pieces.push_back(f);
		while (!intersecting_faces.empty())
		{
			std::vector<MapTypes::Facet*> new_faces;
			MapTypes::Facet* cutter = *(intersecting_faces.begin());
			for (std::size_t j = 0; j < pieces.size(); ++j)
			{
				std::vector<MapTypes::Facet*> tmp = cut(pieces[j], cutter, mesh);
				count_cut++;
				new_faces.insert(new_faces.end(), tmp.begin(), tmp.end());
			}
			pieces = new_faces;
			intersecting_faces.erase(cutter);
		}

		// 2. all the cutting_faces will be cut by f.
		for (std::size_t j = 0; j < cutting_faces.size(); ++j)
		{
			Map::Facet* fa = cutting_faces[j];
			cut(fa, f, mesh);
			count_cut++;
		}
	}
	check_source_planes(mesh);
	MapFacetAttribute<Color> color(mesh, "color");
	FOR_EACH_FACET(Map, mesh, it) color[it] = random_color();
}

// compute the intersection of a plane triplet
// returns true if the intersection exists (p returns the point)
bool HypothesisGenerator::intersection_plane_triplet(const Plane3d* plane1,
	const Plane3d* plane2,
	const Plane3d* plane3,
	vec3& p)
{
	if (plane1 == nil || plane2 == nil || plane3 == nil)
	{
		Logger::err("-") << "null planes" << std::endl;
		return false;
	}
	if (plane1 == plane2 || plane2 == plane3)
	{
		Logger::err("-") << "identical planes" << std::endl;
		return false;
	}
	Plane_3 p1 = to_cgal_plane(*plane1);
	Plane_3 p2 = to_cgal_plane(*plane2);
	Plane_3 p3 = to_cgal_plane(*plane3);

	CGAL::Object obj = CGAL::intersection(p1, p2, p3);

	// pt is the intersection point of the 3 planes 
	if (const Point_3* pt = CGAL::object_cast<Point_3>(&obj))
	{
		p = to_my_point(*pt);
		return true;
	}
	else if (const Plane_3* plane = CGAL::object_cast<Plane_3>(&obj))
	{
		Logger::warn("-") << "3 faces lie on the same supporting plane" << std::endl;
		return false;
	}
	else if (const Line_3* line = CGAL::object_cast<Line_3>(&obj))
	{
		return false;
	}

	return false;
}

void HypothesisGenerator::triplet_intersection(const std::vector<Plane3d*>& supporting_planes)
{
	triplet_intersection_.clear();

	std::vector<Plane3d*> all_planes = supporting_planes;
	std::sort(all_planes.begin(), all_planes.end());

	for (std::size_t i = 0; i < all_planes.size(); ++i)
	{
		Plane3d* plane1 = all_planes[i];
		for (std::size_t j = i + 1; j < all_planes.size(); ++j)
		{
			Plane3d* plane2 = all_planes[j];
			for (std::size_t k = j + 1; k < all_planes.size(); ++k)
			{
				Plane3d* plane3 = all_planes[k];

				assert(plane1 < plane2);
				assert(plane2 < plane3);

				vec3 p;

				if (intersection_plane_triplet(plane1, plane2, plane3, p))
				{
					triplet_intersection_[plane1][plane2][plane3] = p; // store the intersection in our data base
				}

			}
		}
	}

}

bool HypothesisGenerator::query_intersection(Plane3d* plane1, Plane3d* plane2, Plane3d* plane3, vec3& p)
{
	Plane3d* min_plane = ogf_min(plane1, plane2, plane3);
	Plane3d* max_plane = ogf_max(plane1, plane2, plane3);

	Plane3d* mid_plane = 0;
	if (plane1 != min_plane && plane1 != max_plane)
		mid_plane = plane1;
	else if (plane2 != min_plane && plane2 != max_plane)
		mid_plane = plane2;
	else if (plane3 != min_plane && plane3 != max_plane)
		mid_plane = plane3;

	if (triplet_intersection_.find(min_plane) == triplet_intersection_.end())
		return false;

	std::map<Plane3d*, std::map<Plane3d*, vec3> >& tmp2 = triplet_intersection_[min_plane];
	if (tmp2.find(mid_plane) == tmp2.end())
		return false;

	std::map<Plane3d*, vec3>& tmp3 = tmp2[mid_plane];
	if (tmp3.find(max_plane) == tmp3.end())
		return false;

	p = tmp3[max_plane];
	return true;
}

Map* HypothesisGenerator::generate(PolyFitInfo* polyfit_info, Map::Facet* foot_print, const std::vector<vec3>& line_segments)
{
	if (!pset_)
		return nil;

	if (pset_->groups().empty())
	{
		Logger::warn("-") << "planar segments do not exist" << std::endl;
		return nil;
	}
	collect_valid_planes(polyfit_info, foot_print, line_segments);
	Map* mesh = compute_proxy_mesh(polyfit_info, foot_print);

	if (!mesh)
		return nil;
	facet_attrib_supporting_vertex_group_.bind(mesh, Method::facet_attrib_supporting_vertex_group);
	edge_source_planes_.bind(mesh, "EdgeSourcePlanes");
	vertex_source_planes_.bind(mesh, "VertexSourcePlanes");
	facet_attrib_supporting_plane_.bind(mesh, "FacetSupportingPlane");
	triplet_intersection(polyfit_info->planes);
	pairwise_cut(mesh);


#if REMOVE_DEGENERATE_FACES
	remove_degenerated_facets(mesh);
#endif

	facet_attrib_supporting_vertex_group_.unbind();
	facet_attrib_supporting_plane_.unbind();
	edge_source_planes_.unbind();
	vertex_source_planes_.unbind();
	check_source_planes(mesh);

	Logger::out("    -") << "num candidate faces: " << mesh->size_of_facets() << std::endl;

	return mesh;
}

Map* HypothesisGenerator::generate(PolyFitInfo* polyfit_info,
	Map* inter_result,
	Map::Facet* foot_print,
	const std::vector<vec3>& line_segments)
{
	if (!pset_)
		return nil;

	if (pset_->groups().empty())
	{
		Logger::warn("-") << "planar segments do not exist" << std::endl;
		return nil;
	}
	collect_valid_planes(polyfit_info, foot_print, line_segments);
	Map* mesh = compute_proxy_mesh(polyfit_info, foot_print);

	if (!mesh)
		return nil;
	facet_attrib_supporting_vertex_group_.bind(mesh, Method::facet_attrib_supporting_vertex_group);
	edge_source_planes_.bind(mesh, "EdgeSourcePlanes");
	vertex_source_planes_.bind(mesh, "VertexSourcePlanes");
	facet_attrib_supporting_plane_.bind(mesh, "FacetSupportingPlane");
	triplet_intersection(polyfit_info->planes);
	pairwise_cut(mesh);

#if REMOVE_DEGENERATE_FACES
	remove_degenerated_facets(mesh);
#endif

	facet_attrib_supporting_vertex_group_.unbind();
	facet_attrib_supporting_plane_.unbind();
	edge_source_planes_.unbind();
	vertex_source_planes_.unbind();
	check_source_planes(mesh);

	Logger::out("    -") << "num candidate faces: " << mesh->size_of_facets() << std::endl;

	return mesh;
}


