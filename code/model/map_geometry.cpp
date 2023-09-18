/*
*  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
*  Copyright (C) 2000-2005 INRIA - Project ALICE
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*  If you modify this software, you should include a notice giving the
*  name of the person performing the modification, the date of modification,
*  and the reason for such modification.
*
*  Contact: Bruno Levy - levy@loria.fr
*
*     Project ALICE
*     LORIA, INRIA Lorraine,
*     Campus Scientifique, BP 239
*     54506 VANDOEUVRE LES NANCY CEDEX
*     FRANCE
*
*  Note that the GNU General Public License does not permit incorporating
*  the Software into proprietary programs.
*
* As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
*     Qt, SuperLU, WildMagic and CGAL
*/


#include "map_geometry.h"
#include "map_attributes.h"
#include "map_copier.h"
#include "map_builder.h"
#include "map_circulators.h"
#include "../math/polygon2d.h"

namespace Geom
{

	vec3 facet_normal(const Map::Facet* f)
	{
		vec3 result(0, 0, 0);
		Map::Halfedge* cir = f->halfedge();
		do
		{
			vec3 v0 = vector(cir);
			vec3 v1 = vector(cir->prev()->opposite());
			v0 = normalize(v0);
			v1 = normalize(v1);
			if (std::abs(dot(v0, v1)) < 0.999)
			{
				vec3 n = cross(v0, v1);
				result = result + n;
			}
			cir = cir->next();
		} while (cir != f->halfedge());
		result = normalize(result);
		return result;
	}

	vec3 vertex_normal(const Map::Vertex* v)
	{
		vec3 result(0, 0, 0);
		Map::Halfedge* cir = v->halfedge();
		do
		{
			if (!cir->is_border())
			{
				vec3 v0 = vector(cir->next());
				vec3 v1 = vector(cir->opposite());
				vec3 n = cross(v0, v1);
				result = result + n;
			}
			cir = cir->next_around_vertex();
		} while (cir != v->halfedge());
		result = normalize(result);
		return result;
	}

	vec3 triangle_normal(const Map::Facet* f)
	{
		ogf_assert(f->is_triangle());
		vec3 result = (
			cross(vector(f->halfedge()->next()),
				vector(f->halfedge()->opposite()))
		) + (
			cross(vector(f->halfedge()),
				vector(f->halfedge()->prev()->opposite()))
		) + (
			cross(vector(f->halfedge()->next()->next()),
				vector(f->halfedge()->next()->opposite()))
		);
		result = normalize(result);
		return result;
	}

	Plane3d facet_plane(const Map::Facet* f)
	{
		return Plane3d(
			f->halfedge()->vertex()->point(),
			f->halfedge()->next()->vertex()->point(),
			f->halfedge()->next()->next()->vertex()->point()
		);
	}

	Polygon3d facet_polygon(const Map::Facet* f)
	{
		Polygon3d plg;
		Map::Halfedge* cir = f->halfedge();
		do
		{
			plg.push_back(cir->vertex()->point());
			cir = cir->next();
		} while (cir != f->halfedge());
		return plg;
	}

	/*
	// I do not trust this one for the moment ...
	double facet_area(const Map::Facet* f) {
	vec3 n = facet_normal(f) ;
	vec3 w(0,0,0) ;
	Map::Halfedge* it = f->halfedge() ;
	do {
	vec3 v1(
	it-> vertex()-> point().x,
	it-> vertex()-> point().y,
	it-> vertex()-> point().z
	) ;
	vec3 v2(
	it-> next()-> vertex()-> point().x,
	it-> next()-> vertex()-> point().y,
	it-> next()-> vertex()-> point().z
	) ;
	w = w + (v1 ^ v2) ;
	it = it->next() ;
	} while(it != f->halfedge()) ;
	return 0.5 * ::fabs(w * n) ;
	}
	*/

	double facet_area(const Map::Facet* f)
	{
		double result = 0;
		Map::Halfedge* h = f->halfedge();
		const vec3& p = h->vertex()->point();
		h = h->next();
		do
		{
			result += triangle_area(
				p,
				h->vertex()->point(),
				h->next()->vertex()->point()
			);
			h = h->next();
		} while (h != f->halfedge());
		return result;
	}

	double border_length(Map::Halfedge* start)
	{
		ogf_assert(start->is_border());
		double result = 0;
		Map::Halfedge* cur = start;
		do
		{
			result += edge_length(cur);
			cur = cur->next();
		} while (cur != start);
		return result;
	}

	double map_area(const Map* map)
	{
		double result = 0;
		FOR_EACH_FACET_CONST(Map, map, it)
		{
			result += facet_area(it);
		}
		return result;
	}

	Box3d bounding_box(const Map* map)
	{
		Box3d result;
		FOR_EACH_VERTEX_CONST(Map, map, it)
		{
			result.add_point(it->point());
		}
		return result;
	}

	Map* duplicate(const Map* map)
	{
		if (!map)
			return nil;

		Map* result = new Map;

		if (result != nil)
		{
			MapBuilder builder(result);
			MapCopier copier;
			copier.set_copy_all_attributes(true);
			builder.begin_surface();
			copier.copy(builder, const_cast<Map*>(map));
			builder.end_surface();
		}
		return result;
	}

	Map* copy_to_mesh(const Polygon3d& plg)
	{
		if (plg.size() < 3)
		{
			std::cout << "plg.size() < 3" << std::endl;
			return nil;
		}

		//////////////////////////////////////////////////////////////////////////

		Map* mesh = new Map;
		MapBuilder builder(mesh);
		builder.begin_surface();

		for (unsigned int i = 0; i < plg.size(); ++i)
		{
			builder.add_vertex(plg[i]);
		}

		builder.begin_facet();
		for (unsigned int i = 0; i < plg.size(); ++i)
			builder.add_vertex_to_facet(i);;
		builder.end_facet();
		builder.end_surface();

		mesh->compute_facet_normals();
		return mesh;
	}

	Map* copy_to_mesh(const std::vector<Polygon3d>& plgs)
	{
		if (plgs.empty())
		{
			std::cout << "plgs.empty()" << std::endl;
			return nil;
		}

		//////////////////////////////////////////////////////////////////////////

		Map* mesh = new Map;
		MapBuilder builder(mesh);
		builder.begin_surface();

		int id = 0;
		for (std::size_t j = 0; j < plgs.size(); ++j)
		{
			const Polygon3d& plg = plgs[j];
			builder.begin_facet();
			for (unsigned int i = 0; i < plg.size(); ++i)
			{
				builder.add_vertex(plg[i]);
				builder.add_vertex_to_facet(id);
				++id;
			}
			builder.end_facet();
		}
		builder.end_surface();

		mesh->compute_facet_normals();
		return mesh;
	}

	Map* merge(const std::vector<Map*>& maps)
	{
		if (maps.empty())
			return nil;

		MapCopier copier;
		copier.set_copy_all_attributes(true);

		Map* mesh = new Map;
		MapBuilder builder(mesh);

		builder.begin_surface();
		int cur_vertex_id = 0;
		int cur_tex_vertex_id = 0;
		for (unsigned int i = 0; i < maps.size(); ++i)
		{
			Map* cur = maps[i];
			if (!cur)
				continue;

			MapVertexAttribute<int> vertex_id(cur);
			copier.copy(builder, cur, vertex_id, cur_vertex_id);
		}

		builder.end_surface();

		mesh->compute_facet_normals();
		return mesh;
	}

	void merge_into_source(Map* source, Map* another)
	{
		if (!source)
			return;

		if (!another)
			return;

		int cur_vertex_id = 0;
		MapVertexAttribute<int> vertex_id(another);

		MapCopier copier;
		copier.set_copy_all_attributes(true);
		MapBuilder builder(source);
		builder.begin_surface();
		copier.copy(builder, another, vertex_id, cur_vertex_id);
		builder.end_surface();

		source->compute_facet_normals();
	}

    void	merge_into_ground(Map* source, std::vector<vec3> vts)
    {
        if (!source)
            return;

        if (!vts.size())
            return;

        MapBuilder builder(source);
        builder.begin_surface();
        int id =0;
        builder.begin_facet();
        for (auto p:vts)
        {
            builder.add_vertex(p);
            builder.add_vertex_to_facet(id);
            ++id;
        }
        builder.end_facet();
        builder.end_surface();
        source->compute_facet_normals();
    }

	void merge_into_source(Map* source, Map::Facet* f)
	{
		if (!source)
			return;

		if (!f)
			return;

		MapBuilder builder(source);
		builder.begin_surface();
		int id = 0;
		builder.begin_facet();
		FacetHalfedgeCirculator cir(f);
		std::vector<Map::Halfedge*> ring;
		for (; !cir->end(); ++cir)
		{
			Map::Halfedge* h = cir->halfedge();
			ring.push_back(h);
		}
		auto normal = Geom::facet_normal(f);
		if (normal.z > 0)
		{
			for (auto it = ring.rbegin(); it != ring.rend(); ++it)
			{
				auto p = (*it)->vertex()->point();
				builder.add_vertex(p);
				builder.add_vertex_to_facet(id);
                std::cout<<"id:  "<<id<<std::endl;
				++id;

			}
		}
		else
		{
			for (auto it = ring.begin(); it != ring.end(); ++it)
			{
				const vec3& p = (*it)->vertex()->point();
				builder.add_vertex(p);
				builder.add_vertex_to_facet(id);
                std::cout<<"id:  "<<id<<std::endl;
				++id;
			}
		}

		builder.end_facet();
		builder.end_surface();

		source->compute_facet_normals();
	}

}
