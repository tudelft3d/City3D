
#include "prism.h"
#include "map.h"
#include "map_builder.h"
#include "../basic/logger.h"


void Prism::clear() {
	base_polygon_.clear();
	tex_coord_.clear();
}

Prism::~Prism() {
	clear();
}

void Prism::add_point(const vec3& p) {
	base_polygon_.push_back(p);
}

void Prism::set_height(double h, bool bTop) {
	height_ = h;
}

Prism& Prism::operator = (const Prism& prism) {
	clear();

	set_direction(prism.direction()); 
	set_height(prism.height());
	set_base_polygon(prism.base_polygon());

	return *this;
}

Plane3d Prism::top_plane() const {
	vec3 offset = direction_*height_;
	return Plane3d(base_polygon_[0] + offset, base_polygon_[1] + offset, base_polygon_[2] + offset);
}


Plane3d Prism::base_plane() const {
	return Plane3d(base_polygon_[0], base_polygon_[1], base_polygon_[2]);
}


Map* Prism::copy_to_mesh(bool include_ends) const  {
	if (base_polygon_.size() < 3) {
		Logger::out("-") << "base_polygon_.size() < 3" << std::endl;
		return nil;
	}

	if (!tex_coord_.empty() && tex_coord_.size() != base_polygon_.size()) {
		Logger::out("-") << "tex_coord_.size() != base_polygon_.size()" << std::endl;
		return nil;
	}

	std::vector<vec3> base_plg = base_polygon_;
	std::vector<vec2> text_plg(tex_coord_.begin(), tex_coord_.end());

	//////////////////////////////////////////////////////////////////////////
	// in case the reverse direction
	vec3 v12 = base_plg[2] - base_plg[1];
	vec3 v10 = base_plg[0] - base_plg[1];
	vec3 n = cross(v12, v10);
	vec3 dir = direction_;
	//if (dot(n, direction_) < 0) {
	//	vec3 trans = direction_ * height_;
	//	for (unsigned int i=0; i<base_plg.size(); ++i) {
	//		base_plg[i] = base_plg[i] + trans;
	//		dir = -direction_;
	//	}
	//}
	//////////////////////////////////////////////////////////////////////////

	// prepare the vertices;
	std::vector<vec3> base_vts = base_plg;
	std::vector<vec3> top_vts = base_plg;
	vec3 trans = dir * height_;
	for (unsigned int i=0; i<top_vts.size(); ++i) {
		top_vts[i] = top_vts[i] + trans;
	}

	//////////////////////////////////////////////////////////////////////////

	Map* mesh = new Map;
	MapBuilder builder(mesh);
	builder.begin_surface();

	for (unsigned int i=0; i<base_vts.size(); ++i) {
		builder.add_vertex(base_vts[i]);
		//if (!text_plg.empty()) 
		//	builder.add_tex_vertex(text_plg[i]);
	}
	for (unsigned int i=0; i<top_vts.size(); ++i) {
		builder.add_vertex(top_vts[i]);
		//if (!text_plg.empty()) 
		//	builder.add_tex_vertex(text_plg[i]);
	}

	// base polygon
	if (include_ends) {
		builder.begin_facet();
		for (unsigned int i = 0; i < base_vts.size(); ++i) {
			unsigned int id = (base_vts.size() - 1) - i;
			builder.add_vertex_to_facet(id);

			//if (!text_plg.empty())
			//	builder.set_corner_tex_vertex(id);
		}
		builder.end_facet();
	}

	// top polygon
	unsigned int offset = base_vts.size();
	if (include_ends) {
		builder.begin_facet();
		for (unsigned int i = 0; i < top_vts.size(); ++i) {
			unsigned int id = i + offset;
			builder.add_vertex_to_facet(id);

			//if (!text_plg.empty())
			//	builder.set_corner_tex_vertex(id);
		}
		builder.end_facet();
	}

	// side polygons;
	for (unsigned int i=0; i<base_vts.size(); ++i) {
		builder.begin_facet();
		builder.add_vertex_to_facet(i);
		builder.add_vertex_to_facet((i+1)%base_vts.size());
		builder.add_vertex_to_facet((i+1)%base_vts.size() + offset);
		builder.add_vertex_to_facet(i + offset);
		builder.end_facet();
	}

	builder.end_surface();

	mesh->compute_facet_normals();
	return mesh;
}