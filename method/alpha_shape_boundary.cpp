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


#include "alpha_shape_boundary.h"
#include "../basic/logger.h"
#include "alpha_shape.h"
#include "../model/kdtree_search.h"
#include "regularize_polygon.h"
#include <CGAL/Shape_regularization/regularize_contours.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Polygon_2_algorithms.h>
#include"intersection_area.h"


namespace PS = CGAL::Polyline_simplification_2;
typedef PS::Stop_below_count_ratio_threshold Stop;
typedef PS::Squared_distance_cost Cost;

using Contour = std::vector<Point_2>;
using Contour_directions =
        CGAL::Shape_regularization::Contours::Multiple_directions_2<Kernel, Contour>;

struct vec2_ind {
    int data[2];

    float operator[](int idx) const
    { return data[idx]; }
};

vec2_ind find_edges_j(std::vector<vec2_ind> &edge_sets, int j)
{
    for (auto edge = edge_sets.begin(); edge != edge_sets.end(); edge++)
    {
        if (edge->data[0] == j || edge->data[1] == j)
        {
            vec2_ind target = *edge;
            edge_sets.erase(edge);
            return target;
        }
    }
}

std::vector<int> orient_ply(std::vector<vec2_ind> edges)
{
    std::vector<vec2_ind> new_edges;
    std::set<int> index_vertices;
    for (auto edge = edges.begin(); edge != edges.end(); edge++)
    {
        index_vertices.insert(edge->data[0]);
        index_vertices.insert(edge->data[1]);
    }
    for (auto num: index_vertices)
    {
        int count = 0;
        for (int i = 0; i < edges.size(); ++i)
        {
            auto a = edges[i].data[0];
            auto b = edges[i].data[1];
            if (a == num || b == num)
            {
                count++;
            }
        }
        if (count != 2)
            std::cerr << "the boundary is not simple" << std::endl;

    }
    //start from here
    vec2_ind last_edge = edges.back();
    new_edges.push_back(last_edge);
    edges.pop_back();
    while (!edges.empty())
    {
        auto i = last_edge.data[0];
        auto j = last_edge.data[1];
        auto target_edge = find_edges_j(edges, j);
        if (target_edge.data[0] == j)
        {
            last_edge = target_edge;
        } else
        {
            vec2_ind new_target({target_edge.data[1], target_edge.data[0]});
            last_edge = new_target;
        }
        new_edges.push_back(last_edge);
    }
    std::vector<int> ply_index;
    ply_index.push_back(new_edges[0].data[0]);
    ply_index.push_back(new_edges[0].data[1]);
    for (int i = 1; i < new_edges.size() - 1; ++i)
    {
        ply_index.push_back(new_edges[i].data[1]);
    }
    return ply_index;
}


std::vector<vec2> regulaize_cgal(std::vector<Point_2> contour)
{

    // Set parameters.
    const FT min_length_2 = FT(1.5);
    const FT max_angle_2 = FT(20);
    const FT max_offset_2 = FT(2.0);

    // Regularize.
    const bool is_closed = true;
    Contour_directions directions(
            contour, is_closed, CGAL::parameters::
            minimum_length(min_length_2).maximum_angle(max_angle_2));

    std::vector<Point_2> regularized;
    CGAL::Shape_regularization::Contours::regularize_closed_contour(
            contour, directions, std::back_inserter(regularized),
            CGAL::parameters::maximum_offset(max_offset_2));
    std::vector<vec2> results;

    for (int i = 0; i < regularized.size(); ++i)
    {
        results.push_back({regularized[i].x(), regularized[i].y()});
    }
    return results;
}

std::vector<vec2> regularize_segments(std::vector<Point_2> contour)
{

    //simplify the contour
    Cost cost;
    Polygon_2 polygon(contour.begin(), contour.end());
    FT thre = 0.3;
    while (polygon.size() > 60)
    {
        polygon = PS::simplify(polygon, cost, Stop(thre));
        thre *= 0.9;
    }
    //get the simplified contour from polygon
    contour.clear();
    for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it)
    {
        contour.push_back(Point_2(*it));
    }
    std::vector<vec2> plys;
    for (int i = 0; i < contour.size(); ++i)
    {
        plys.push_back(vec2(contour[i].x(), contour[i].y()));
    }

    // Regularize the contour.
    RegularizePolygon rg;

    //compare our method with CGAL's method
    std::vector<vec2> regularized = rg.reg_ply(plys);
    std::vector<vec2> regualized_cgal = regulaize_cgal(contour);
    //choose the better one, check the intersection area;
    double area_cgal = intersection_area(plys, regualized_cgal);
    double area_regualized = intersection_area(plys, regularized);
    if (area_cgal > area_regualized)
    {
        regularized = regualized_cgal;
    }
    //remove the collinear points
    std::vector<vec2> regularized_copy=regularized;
    std::vector<bool> check_linear(regularized.size(), false);
    for (int i = 0; i < regularized.size(); ++i)
    {
        int id = i, ip = (i + 1) % regularized.size(), im = (i + 2) % regularized.size();
        auto v0 = regularized[id], v1 = regularized[ip], v2 = regularized[im];
        //compute the cross product of v0v1 and v0v2
        auto v01 = normalize((v0 - v1)), v12 = normalize((v1 - v2));
        double cos = std::abs(dot(v01, v12));
        if (cos > 0.99)
        {
            check_linear[ip] = true;
        }
    }
    regularized.clear();
    for (int i = 0; i < regularized_copy.size(); ++i)
    {
        if (!check_linear[i])
        {
            regularized.push_back(regularized_copy[i]);
        }
    }

    return regularized;
}


std::vector<vec2> AlphaShapeBoundary::apply(PointSet *pset, double radius)
{
    std::size_t num_input = pset->num_points();
    const std::vector<vec3> &points = pset->points();
    std::list<Point_2> pts;
    for (std::size_t i = 0; i < num_input; ++i)
    {
        const vec3 &p = points[i];
        vec2 q(p.x, p.y);
        const Point_2 &qq = to_cgal_point(q);
        pts.push_back(qq);
    }
    double alpha = radius * radius;
    AlphaShape as(pts.begin(), pts.end());
    auto opt_alpha = *as.find_optimal_alpha(1);
    if (alpha < opt_alpha)
    {
        alpha = opt_alpha;
    }
    as.set_alpha(alpha);
    //////////////////////////////////////////////////////////////////////////
    auto eit = as.finite_edges_begin();
    std::vector<vec2_ind> edges;
    for (; eit != as.finite_edges_end(); ++eit)
    {
        if (as.classify(*eit) == AlphaShape::REGULAR)
        {
            auto e = *eit;
            auto i1 = e.first->vertex((e.second + 1) % 3)->index();
            auto i2 = e.first->vertex((e.second + 2) % 3)->index();
            edges.push_back(vec2_ind({i1, i2}));
        }
    }
    auto ply = orient_ply(edges);
    std::vector<Point_2> ply_points;
    for (int i = 0; i < ply.size(); ++i)
    {
        vec2 pi(points[ply[i]].x, points[ply[i]].y);
        ply_points.push_back(to_cgal_point(pi));
    }
    return regularize_segments(ply_points);;
}

