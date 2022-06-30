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

#include "face_selection.h"
#include "polyfit_info.h"
#include "method_global.h"
#include "../model/point_set.h"
#include "../model/map_editor.h"
#include "../model/map_geometry.h"
#include "../model/map_circulators.h"
#include "../basic/logger.h"
#include "../model/map_builder.h"
#include "../model/map_io.h"

std::vector<Map::Facet*> FaceSelection::overlapping_faces(Map::Facet* f,
                                                          const std::vector<Map::Facet*>& faces,
                                                          Map::Facet* foot_print)
{
	vec3 c(0, 0, 0);
	int degree = 0;

	FacetHalfedgeCirculator cir(f);
	for (; !cir->end(); ++cir)
	{
		c += cir->halfedge()->vertex()->point();
		++degree;
	}
	c /= degree;

	const Plane3d& plane = Geom::facet_plane(foot_print);
	c = plane.projection(c);
	const vec2& p = plane.to_2d(c);

	std::vector<Map::Facet*> roofs({ f });

	for (std::size_t i = 0; i < faces.size(); ++i)
	{
		Map::Facet* face = faces[i];
		if (f == face)
			continue;

		Polygon2d plg;
		FacetHalfedgeCirculator fcir(face);
		for (; !fcir->end(); ++fcir)
		{
			vec3 q = fcir->halfedge()->vertex()->point();
			q = plane.projection(q);
			const vec2& r = plane.to_2d(q);
			plg.push_back(r);
		}
		if (Geom::point_is_in_polygon(plg, p))
			roofs.push_back(face);
	}

	return roofs;
}

std::vector<std::vector<Map::Facet*> > FaceSelection::find_multi_roofs(Map* mesh,
	Map::Facet* foot_print,
	std::vector<Plane3d*>& v)
{
	facet_attrib_supporting_plane_.bind_if_defined(model_, "FacetSupportingPlane");
	std::vector<Map::Facet*> faces;
	FOR_EACH_FACET(Map, mesh, it)
	{
		int count = 0; //count the testing times
		for (std::size_t k = 0; k < v.size(); k++)
		{
			const Plane3d* ver = v[k];
			if (facet_attrib_supporting_plane_[it] == ver)
				break;
			count++;
		}
		if (count == v.size())
			faces.push_back(it);
	}
	std::vector<std::vector<Map::Facet*> > multiple_roofs;
	for (std::size_t i = 0; i < faces.size(); ++i)
	{
		Map::Facet* ft = faces[i];
		const std::vector<Map::Facet*>& roofs = overlapping_faces(ft, faces, foot_print);
		if (roofs.size() > 1)
			multiple_roofs.push_back(roofs);
	}
	return multiple_roofs;
}

void FaceSelection::optimize(PolyFitInfo* polyfit_info,
	Map::Facet* foot_print,
	std::vector<Plane3d*>& v, LinearProgramSolver::SolverName solver_name)
{
	if (pset_ == 0 || model_ == 0)
		return;

	facet_attrib_supporting_vertex_group_.bind_if_defined(model_, Method::facet_attrib_supporting_vertex_group);
	if (!facet_attrib_supporting_vertex_group_.is_bound())
	{
		Logger::err("-") << "attribute " << Method::facet_attrib_supporting_vertex_group << " doesn't exist"
						 << std::endl;
		return;
	}
	facet_attrib_supporting_point_num_.bind_if_defined(model_, Method::facet_attrib_supporting_point_num);
	if (!facet_attrib_supporting_point_num_.is_bound())
	{
		Logger::err("-") << "attribute " << Method::facet_attrib_supporting_point_num << " doesn't exist" << std::endl;
		return;
	}
	facet_attrib_facet_area_.bind_if_defined(model_, Method::facet_attrib_facet_area);
	if (!facet_attrib_facet_area_.is_bound())
	{
		Logger::err("-") << "attribute " << Method::facet_attrib_facet_area << " doesn't exist" << std::endl;
		return;
	}
	facet_attrib_covered_area_.bind_if_defined(model_, Method::facet_attrib_covered_area);
	if (!facet_attrib_covered_area_.is_bound())
	{
		Logger::err("-") << "attribute " << Method::facet_attrib_covered_area << " doesn't exist" << std::endl;
		return;
	}

	facet_attrib_supporting_plane_.bind_if_defined(model_, "FacetSupportingPlane");

	MapFacetAttribute<bool> is_confident_facet(model_, "is_confident_facet");


	//////////////////////////////////////////////////////////////////////////

	double total_points = double(pset_->points().size());
	std::size_t idx = 0;
	MapFacetAttribute<std::size_t> facet_indices(model_);
	FOR_EACH_FACET(Map, model_, it)
	{
		Map::Facet* f = it;
		facet_indices[f] = idx;
		++idx;
	}
    //StopWatch w;
	const std::vector<FaceStar>& fans = adjacency_.extract(model_, polyfit_info->planes);
	MapHalfedgeAttribute<bool> is_bound(model_);

	FOR_EACH_HALFEDGE(Map, model_, it)
	{
		auto edge = *it;
		is_bound[edge] = false;
	}
	for (std::size_t i = 0; i < fans.size(); ++i)
	{
		const FaceStar& star = fans[i];
		if (star.size() == 1)
		{
			Map::Halfedge* h = *star.begin();
			is_bound[h] = true;
		}
	}

	std::size_t num_faces = model_->size_of_facets();
	std::size_t num_edges = 0;
	double bbox_zmax = model_->bbox().z_max();
	double bbox_zheight = model_->bbox().z_max() - model_->bbox().z_min();
	std::map<const FaceStar*, std::size_t> edge_usage_status;    // keep or remove an intersecting edges
	for (std::size_t i = 0; i < fans.size(); ++i)
	{
		const FaceStar& fan = fans[i];
		if (fan.size() == 4)
		{
			std::size_t var_idx = num_faces + num_edges;
			edge_usage_status[&fan] = var_idx;
			++num_edges;
		}
	}
    double coeff_data_fitting = Method::lambda_data_fitting;
	double coeff_hight = total_points * Method::lambda_model_height / bbox_zheight;
	coeff_hight /= num_faces;
	double coeff_complexity = total_points * Method::lambda_model_complexity / double(fans.size());

	typedef Variable<double> Variable;
	typedef LinearExpression<double> Objective;
	typedef LinearConstraint<double> Constraint;
	typedef LinearProgram<double> LinearProgram;
	Objective obj;

	std::map<const FaceStar*, std::size_t> edge_sharp_status;    // the edge is sharp or not
	std::size_t num_sharp_edges = 0;
	for (std::size_t i = 0; i < fans.size(); ++i)
	{
		const FaceStar& fan = fans[i];
		if (fan.size() == 4)
		{
			std::size_t var_idx = num_faces + num_edges + num_sharp_edges;
			edge_sharp_status[&fan] = var_idx;
			// accumulate model complexity term
			obj.add_coefficient(var_idx, coeff_complexity);
			++num_sharp_edges;
		}
	}
	assert(num_edges == num_sharp_edges);

	FOR_EACH_FACET(Map, model_, it)
	{
		Map::Facet* f = it;
		std::size_t var_idx = facet_indices[f];
		vec3 c(0, 0, 0);
		int degree = 0;
		FacetHalfedgeCirculator cir(f);
		for (; !cir->end(); ++cir)
		{
			c += cir->halfedge()->vertex()->point();
			++degree;
		}
		c /= degree;
		double avg_z = bbox_zmax - c.z;
		// accumulate data fitting term
		double num = facet_attrib_supporting_point_num_[f];

		obj.add_coefficient(var_idx, -coeff_data_fitting * num);
		// accumulate face height  term
		obj.add_coefficient(var_idx,  coeff_hight * avg_z);

		auto v_plane = facet_attrib_supporting_plane_[f];
		bool v_face = false;
		for (int i = 0; i < v.size(); ++i)
		{
			if (v_plane == v[i])
			{
				v_face = true;
				break;
			}
		}
		double uncovered_area = (facet_attrib_facet_area_[f]);
   // accumulate vertical face area term,
        double coeff_vertical_coverage = 0.1*total_points * Method::lambda_model_complexity / model_->bbox().area();
        if (v_face)
		{
			obj.add_coefficient(var_idx, coeff_vertical_coverage * uncovered_area);
		}
	}
	program_.set_objective(obj, LinearProgram::MINIMIZE);

	std::size_t total_variables = num_faces + num_edges + num_sharp_edges;
	typedef LinearProgram::Variable Variable;
	for (std::size_t i = 0; i < total_variables; ++i)
	{
		program_.add_variable(Variable(Variable::BINARY));
	}

	//////////////////////////////////////////////////////////////////////////

	typedef LinearProgram::Constraint Constraint;

	// Add constraints: the number of faces associated with an edge must be either 2 or 0
	std::size_t var_edge_used_idx = 0;
	for (std::size_t i = 0; i < fans.size(); ++i)
	{
		const FaceStar& fan = fans[i];

		if (fan.size() > 2)
		{
			Constraint constraint;
			for (std::size_t j = 0; j < fan.size(); ++j)
			{
				MapTypes::Facet* f = fan[j]->facet();
				std::size_t var_idx = facet_indices[f];
				constraint.add_coefficient(var_idx, 1.0);
			}

			std::size_t var_idx = num_faces + var_edge_used_idx;
			constraint.add_coefficient(var_idx, -2.0);  // 
			++var_edge_used_idx;
			constraint.set_bounds(Constraint::FIXED, 0.0, 0.0);
			program_.add_constraint(constraint);
		}
	}

	FOR_EACH_FACET(Map, model_, it)
	{
		Map::Facet* f = it;
		auto v_plane = facet_attrib_supporting_plane_[f];
		bool v_face = false;
		for (int i = 0; i < v.size(); ++i)
		{
			if (v_plane == v[i])
			{
				v_face = true;
				break;
			}
		}
//exclude the added vertical planar segments
		if (!v_face && is_confident_facet[f] )
		{
			Constraint constraint;
			std::size_t var_idx1 = facet_indices[f];
			constraint.add_coefficient(var_idx1, 1.0);
			constraint.set_bounds(Constraint::FIXED, 1.0, 1.0);
			program_.add_constraint(constraint);

		}
	}

	// Add constraints: for the sharp edges
	double M = 1.0;
	for (std::size_t i = 0; i < fans.size(); ++i)
	{
		const FaceStar& fan = fans[i];
		if (fan.size() != 4)
			continue;

		Constraint edge_used_constraint;
		std::size_t var_edge_usage_idx = edge_usage_status[&fan];
		edge_used_constraint.add_coefficient(var_edge_usage_idx, 1.0);

		std::size_t var_edge_sharp_idx = edge_sharp_status[&fan];
		edge_used_constraint.add_coefficient(var_edge_sharp_idx, -1.0);

		edge_used_constraint.set_bounds(Constraint::LOWER, 0.0, 0.0);
		program_.add_constraint(edge_used_constraint);

		for (std::size_t j = 0; j < fan.size(); ++j)
		{
			MapTypes::Facet* f1 = fan[j]->facet();
			Plane3d* plane1 = facet_attrib_supporting_plane_[f1];
			std::size_t fid1 = facet_indices[f1];
			for (std::size_t k = j + 1; k < fan.size(); ++k)
			{
				MapTypes::Facet* f2 = fan[k]->facet();
				Plane3d* plane2 = facet_attrib_supporting_plane_[f2];
				std::size_t fid2 = facet_indices[f2];

				if (plane1 != plane2)
				{
					Constraint edge_sharp_constraint;
					edge_sharp_constraint.add_coefficient(var_edge_sharp_idx, 1.0);
					edge_sharp_constraint.add_coefficient(fid1, -M);
					edge_sharp_constraint.add_coefficient(fid2, -M);
					edge_sharp_constraint.add_coefficient(var_edge_usage_idx, -M);
					edge_sharp_constraint.set_bounds(Constraint::LOWER, 1.0 - 3.0 * M, 0.0);
					program_.add_constraint(edge_sharp_constraint);
				}
			}
		}
	}

	// Add constraints: single-roof
	const std::vector<std::vector<Map::Facet*> >& multiple_roofs = find_multi_roofs(model_, foot_print, v);
	std::set<std::vector<int>> multi_roof_set;
	for (std::size_t i = 0; i < multiple_roofs.size(); ++i)
	{
		const std::vector<Map::Facet*>& roofs = multiple_roofs[i];
		std::set<int> temp;
		std::vector<int> face_index;
		for (std::size_t j = 0; j < roofs.size(); ++j)
		{
			Map::Facet* f = roofs[j];
			std::size_t fid = facet_indices[f];
			temp.insert(fid);
		}
		face_index.insert(face_index.end(), temp.begin(), temp.end());
		multi_roof_set.insert(face_index);

	}

	for (std::set<std::vector<int>>::iterator iter = multi_roof_set.begin(); iter != multi_roof_set.end(); ++iter)
	{
		const std::vector<int>& roofs = *iter;
		Constraint constraint;
		for (std::size_t j = 0; j < roofs.size(); ++j)
		{
			std::size_t fid = roofs[j];
			constraint.add_coefficient(fid, 1.0);
		}
		constraint.set_bounds(Constraint::FIXED, 1.0, 1.0);
		program_.add_constraint(constraint);
	}


	std::vector<std::size_t> vertical_group;
	FOR_EACH_FACET(Map, model_, it)
	{
		Map::Facet* f = it;
		bool vertical_facet = false;
		for (std::size_t k = 0; k < v.size(); k++)
		{
			if (facet_attrib_supporting_plane_[it] == v[k])
			{
				vertical_facet = true;
				break;
			}
		}
		if (vertical_facet)
		{
			int count_boundary = 0;
			FacetHalfedgeCirculator circulator(f);
			for (; !circulator->end(); ++circulator)
			{
				Map::Halfedge* f_edge = circulator->halfedge();
				if (is_bound[f_edge])
					count_boundary++;
			}
			if (count_boundary > 2)
			{
				std::size_t v_index = facet_indices[f];
				auto fplg = f->to_polygon();
				vertical_group.push_back(v_index);
			}
		}
	}
 // the boundary vertical facets cannot be selected
	for (int l = 0; l < vertical_group.size(); ++l)
	{
		Constraint v_constraint;
		std::size_t fid = vertical_group[l];
		v_constraint.add_coefficient(fid, 1.0);
		v_constraint.set_bounds(Constraint::FIXED, 0.0, 0.0);
		program_.add_constraint(v_constraint);
	}
	//////////////////////////////////////////////////////////////////////////

	LinearProgramSolver solver;
	if (solver.solve(&program_, solver_name))
	{

		// mark results
		const std::vector<double>& X = solver.get_result();
		std::vector<Map::Facet*> to_delete;
		FOR_EACH_FACET(Map, model_, it)
		{
			Map::Facet* f = it;
			std::size_t idx = facet_indices[f];
			if (static_cast<int>(std::round(X[idx])) == 0)
			{
				to_delete.push_back(f);
			}
		}

		MapEditor editor(model_);
		for (std::size_t i = 0; i < to_delete.size(); ++i)
		{
			Map::Facet* f = to_delete[i];
			editor.erase_facet(f->halfedge());
		}

		//////////////////////////////////////////////////////////////////////////

		// mark the sharp edges
		MapHalfedgeAttribute<bool> edge_is_sharp(model_, "SharpEdge");
		FOR_EACH_EDGE(Map, model_, it) edge_is_sharp[it] = false;

		for (std::size_t i = 0; i < fans.size(); ++i)
		{
			const FaceStar& fan = fans[i];
			if (fan.size() != 4)
				continue;

			std::size_t idx_sharp_var = edge_sharp_status[&fan];
			if (static_cast<int>(X[idx_sharp_var]) == 1)
			{
				for (std::size_t j = 0; j < fan.size(); ++j)
				{
					Map::Halfedge* e = fan[j];
					Map::Facet* f = e->facet();
					if (f)
					{ // some faces may be deleted
						std::size_t fid = facet_indices[f];
						if (static_cast<int>(X[fid]) == 1)
						{
							edge_is_sharp[e] = true;
							break;
						}
					}
				}
			}
		}

		MapFacetNormal normal(model_);
		FOR_EACH_FACET(Map, model_, it)
		{
			normal[it] = facet_attrib_supporting_plane_[it]->normal();
		}
	}
	else
	{
		Logger::out("-") << "solving the binary program failed." << std::endl;
	}

	facet_attrib_supporting_vertex_group_.unbind();
	facet_attrib_supporting_point_num_.unbind();
	facet_attrib_facet_area_.unbind();
	facet_attrib_covered_area_.unbind();
	facet_attrib_supporting_plane_.unbind();
}
