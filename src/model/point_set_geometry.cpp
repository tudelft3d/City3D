#include "point_set_geometry.h"
#include "point_set.h"
#include "vertex_group.h"
#include "../basic/logger.h"
#include "../basic/progress.h"


PointSetNormalizer::PointSetNormalizer(PointSet* pset) 
	: pset_(pset)
	, normalized_radius_(1.0f)
{
	center_ = vec3(0.0f, 0.0f, 0.0f);

	const std::vector<vec3>& points = pset_->points();
	for (std::size_t i = 0; i < points.size(); ++i) {
		center_ += points[i];
	}
	center_ /= points.size();

	radius_ = 0.0f;
	for (std::size_t i = 0; i < points.size(); ++i) {
		vec3 v = points[i] - center_;
		radius_ = ogf_max(radius_, length(v));
	}
}


void PointSetNormalizer::apply(double normalized_radius) {
	normalized_radius_ = normalized_radius;

	std::vector<vec3>& points = pset_->points();

#pragma omp parallel for
	for (int i = 0; i < points.size(); ++i) {
		points[i] = (normalized_radius_ / radius_) * (points[i] - center_);
	}
}


void PointSetNormalizer::unapply() {
	std::vector<vec3>& points = pset_->points();

#pragma omp parallel for
	for (int i = 0; i < points.size(); ++i) {
		const vec3& p = points[i];
        vec3 v = (radius_ / normalized_radius_) * p;
		points[i] = center_ + v;
	}
}

//________________________________________________________________________


namespace Geom {

	VertexGroup* copy_vertex_group(VertexGroup* g, int offset) {
		VertexGroup* group = new VertexGroup;
		group->set_plane(dynamic_cast<VertexGroup*>(g)->plane());
		group->set_color(g->color());
		for (std::size_t i = 0; i < g->size(); ++i)
			group->push_back(g->at(i) + offset);

		return group;
	}


	PointSet* duplicate(const PointSet* from) {
		if (!from)
			return nil;

		PointSet* to = new PointSet;
		to->points() = from->points();
		to->normals() = from->normals();	
		to->colors() = from->colors();

		// -----------------------------------------------------------------------
		// copy vertex groups
		const std::vector<VertexGroup::Ptr>& groups_from = from->groups();
		std::vector<VertexGroup::Ptr>& groups_to = to->groups();
		for (std::size_t i = 0; i < groups_from.size(); ++i) {
			VertexGroup::Ptr old_g = groups_from[i];
			VertexGroup::Ptr new_g = copy_vertex_group(old_g, 0);
			groups_to.push_back(new_g);
		}

		return to;
	}
	
	void reverse_orientation(VertexGroup* group) {
		PointSet* pset = group->point_set();
		if (pset && pset->has_normals()) {
			std::vector<vec3>& normals = pset->normals();

#pragma omp parallel for
			for (int i = 0; i < group->size(); ++i) {
				std::size_t idx = group->at(i);
				normals[idx] = -normals[idx];
			}

			Plane3d plane(group->plane().point(), -group->plane().normal());
			group->set_plane(plane);

			for (std::size_t i = 0; i < group->children().size(); ++i) {
				VertexGroup* chld = group->children()[i];
				reverse_orientation(chld);
			}
		}
	}

	Box3d bounding_box(const PointSet* pset, bool accurate /* = false*/, int samples /* = 200000*/) {
		Box3d result;
		if (pset->num_points() <= 0)
			return result;

		const std::vector<vec3>& points = pset->points();
		std::size_t step = 1;
		if (!accurate && points.size() > samples)
			step = points.size() / samples;
		for (std::size_t i = 0; i < points.size(); i += step) {
			const vec3& p = points[i];
			result.add_point(vec3(p.data()));
		}
		return result;
	}
}
