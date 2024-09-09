﻿#include "reconstruction.h"
#include "hypothesis_generator.h"
#include "face_selection.h"
#include "point_set_normals.h"
#include "cgal_types.h"
#include "point_set_region_growing.h"
#include "../basic/logger.h"
#include "../basic/progress.h"
#include "../basic/stop_watch.h"
#include "../model/kdtree_search.h"
#include "../model/map_geometry.h"
#include "../model/map_io.h"
#include "method_global.h"
#include "../model/map_circulators.h"
#include "../model/map_builder.h"
#include "../method/dbscan.h"
// #include "../renderer/tessellator.h"
// #include "../renderer/mesh_render.h"
#include "alpha_shape_boundary.h"
#include "otr2_edge_simplify.h"
#include "alpha_shape_mesh.h"
#include "../model/point_set_io.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/interpolation_functions.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_3 Point_3d;
typedef CGAL::Projection_traits_xy_3<K> Gt;
typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay_triangulation;
typedef Delaunay_triangulation::Face_handle Face_handle;

double image_x_min;
double image_y_min;
double image_delta_res;

void Reconstruction::segmentation(PointSet *pset, Map *foot_print)
{
    if (!pset)
    {
        Logger::warn("-") << "point cloud data does not exist" << std::endl;
        return;
    }
    if (!foot_print)
    {
        Logger::warn("-") << "foot print data does not exist" << std::endl;
    }
    pset->groups().clear();

//	Logger::out("-") << "building kd-tree..." << std::endl;
    StopWatch w;
    // estimate the best z value
    double avg_z = 0.0;
    FOR_EACH_VERTEX_CONST(Map, foot_print, it) avg_z += it->point().z;
    avg_z /= foot_print->size_of_vertices();

    KdTreeSearch_var kdtree = new KdTreeSearch;
    kdtree->begin();
    // I need a 2D version of the point cloud
    std::vector<vec3> backup_points = pset->points();

    std::vector<vec3> points = pset->points();
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        points[i].z = avg_z;
        kdtree->add_point(&points[i]);
    }
    kdtree->end();
    w.start();

    MapFacetAttribute<VertexGroup::Ptr> buildings(foot_print, "buildings");

    std::vector<VertexGroup::Ptr> &groups = pset->groups();
    ProgressLogger progress(foot_print->size_of_facets());
    FOR_EACH_FACET_CONST(Map, foot_print, it)
    {
        const Polygon3d &plg = it->to_polygon();
        Polygon2d plg2d;
        Box3d box;
        for (std::size_t i = 0; i < plg.size(); ++i)
        {
            const vec3 &p = plg[i];
            plg2d.push_back(vec2(p.x, p.y));
            box.add_point(vec3(p.x, p.y, avg_z));
        }

        std::vector<unsigned int> nbs;
        double r = box.radius();
        kdtree->find_points_in_radius(box.center(), r * r, nbs);

        std::vector<unsigned char> inside(nbs.size(), 0);
#pragma omp parallel for
        for (int i = 0; i < nbs.size(); ++i)
        {
            unsigned int idx = nbs[i];
            vec2 p(points[idx].x, points[idx].y);
            if (Geom::point_is_in_polygon(plg2d, p))
            {
                inside[i] = 1;
            }
        }

        VertexGroup::Ptr g = new VertexGroup;
        for (int i = 0; i < nbs.size(); ++i)
        {
            if (inside[i])
            {
                g->push_back(nbs[i]);
            }
        }
        if (!g->empty())
        {
            g->set_color(random_color());
            groups.push_back(g);
            buildings[it] = g;
        }

        progress.next();
    }

}

int Reconstruction::extract_building_roof(PointSet *pset,
                                           VertexGroup *building,
                                           unsigned int min_support)
{
    std::vector<unsigned int> p_index = *building;
    std::vector<unsigned int> remaining_ind;

    Region_Growing_Dectetor rg;
    const std::vector<VertexGroup::Ptr> &roofs = rg.detect(pset, p_index, min_support);
    for (std::size_t j = 0; j < roofs.size(); ++j)
    {
        const Color &c = random_color();
        roofs[j]->set_color(c);
    }
    building->set_children(roofs);
    //Logger::out("-") << roofs.size() << " roof planes extracted" << std::endl;
    return roofs.size();
}

bool Reconstruction::extract_roofs(PointSet *pset, Map *foot_print)
{
    if (!pset)
    {
        Logger::warn("-") << "point cloud data does not exist" << std::endl;
        return false;
    }

    if (!foot_print)
    {
        Logger::warn("-") << "footprint does not exist. You must either load it or generate it by clicking the 'Segmentation' button" << std::endl;
        return false;
    }

    if (!MapFacetAttribute<VertexGroup::Ptr>::is_defined(foot_print, "buildings"))
    {
        Logger::warn("-") << "please first perform segmentation of the point cloud into buildings" << std::endl;
        return false;
    }
    MapFacetAttribute<VertexGroup::Ptr> buildings(foot_print, "buildings");


    if (!pset->has_normals())
    {
        PointSetNormals::estimate(pset);
    }

    StopWatch t;
    t.start();

    int num = 0;
    ProgressLogger progress(foot_print->size_of_facets());
    FOR_EACH_FACET_CONST(Map, foot_print, it)
    {
        VertexGroup::Ptr g = buildings[it];
        if (!g)
            continue;
        num += extract_building_roof(pset, g, Method::number_region_growing);
        progress.next();
    }

    return num > 0;
}

PointSet *Reconstruction::create_roof_point_set(const PointSet *pset,
                                                const std::vector<VertexGroup::Ptr> &segments, VertexGroup *building)
{
    const std::vector<vec3> &points = pset->points();
    const std::vector<vec3> &normals = pset->normals();
    const std::vector<vec3> &colors = pset->colors();
    PointSet *new_pset = new PointSet;
    std::vector<unsigned int> full_vector = *building;
    std::set<unsigned int> full_set(full_vector.begin(), full_vector.end());
    std::set<unsigned int> planar_set, remaining_set;
    int count = 0;

    for (std::size_t i = 0; i < segments.size(); ++i)
    {
        VertexGroup *vg = segments[i];

        VertexGroup::Ptr group = new VertexGroup;
        group->set_point_set(new_pset);
        group->set_plane(vg->plane());
        group->set_color(vg->color());
        for (int j = 0; j < vg->size(); ++j)
        {
            unsigned int idx = vg->at(j);
            new_pset->points().push_back(points[idx]);

            if (pset->has_normals())
                new_pset->normals().push_back(normals[idx]);
            if (pset->has_colors())
                new_pset->colors().push_back(colors[idx]);

            group->push_back(count);
            planar_set.insert(idx);
            ++count;
        }
        new_pset->groups().push_back(group);
    }
    //add the remaining points.
    std::set_difference(full_set.begin(),
                        full_set.end(),
                        planar_set.begin(),
                        planar_set.end(),
                        inserter(remaining_set, remaining_set.begin()));

    for (auto iter = remaining_set.begin(); iter != remaining_set.end(); ++iter)
    {
        auto in_p = *iter;
        const vec3 &p = points[in_p];
        new_pset->points().push_back(p);
        if (pset->has_normals())
            new_pset->normals().push_back(normals[in_p]);
        if (pset->has_colors())
            new_pset->colors().push_back(colors[in_p]);
    }

    return new_pset;
}

PointSet *Reconstruction::create_projected_point_set(const PointSet *pset, const PointSet *roof)
{

    KdTreeSearch_var kdtree = new KdTreeSearch;
    kdtree->begin();
    // I need a 2D version of the point cloud
    auto z_min = roof->bbox().z_min(), z_max = roof->bbox().z_max();
    auto y_min = roof->bbox().y_min(), y_max = roof->bbox().y_max();
    auto x_min = roof->bbox().x_min(), x_max = roof->bbox().x_max();
    double avg_z = (z_max + z_min) * 0.5;
    x_min -= 1;
    y_min -= 1;
    x_max += 1;
    y_max += 1;
    std::vector<vec3> backup_points = pset->points();

    std::vector<vec3> points = pset->points();
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        points[i].z = avg_z;
        kdtree->add_point(&points[i]);
    }
    kdtree->end();

    Polygon2d plg2d;
    Box3d box;
    std::vector<vec3> plg;
    plg.push_back(vec3(x_min, y_min, avg_z));
    plg.push_back(vec3(x_min, y_max, avg_z));
    plg.push_back(vec3(x_max, y_max, avg_z));
    plg.push_back(vec3(x_max, y_min, avg_z));

    for (std::size_t i = 0; i < plg.size(); ++i)
    {
        vec3 p = plg[i];
        plg2d.push_back(vec2(p.x, p.y));
        box.add_point(vec3(p.x, p.y, avg_z));
    }

    std::vector<unsigned int> nbs;
    double r = box.radius();
    kdtree->find_points_in_radius(box.center(), r * r, nbs);

    std::vector<unsigned char> inside(nbs.size(), 0);
#pragma omp parallel for
    for (int i = 0; i < nbs.size(); ++i)
    {
        unsigned int idx = nbs[i];
        vec2 p(points[idx].x, points[idx].y);
        if (Geom::point_is_in_polygon(plg2d, p))
        {
            inside[i] = 1;
        }
    }

    VertexGroup::Ptr g = new VertexGroup;
    for (int i = 0; i < nbs.size(); ++i)
    {
        if (inside[i])
        {
            g->push_back(nbs[i]);
        }
    }

    PointSet *new_pset = new PointSet;
    int count = 0;
    VertexGroup::Ptr group = new VertexGroup;
    group->set_point_set(new_pset);
    for (int j = 0; j < g->size(); ++j)
    {
        int idx = g->at(j);
        auto p = backup_points[idx];
        if (p.z < z_max)
        {
            new_pset->points().push_back(p);
            group->push_back(count);
            ++count;
        }
    }
    new_pset->groups().push_back(group);

    return new_pset;
}

// create a point set from a set of planar segments

bool is_simple_polygon(const Map::Facet *f)
{
    const Plane3d &plane = Geom::facet_plane(f);
    Polygon_2 plg;
    FacetHalfedgeConstCirculator cir(f);
    for (; !cir->end(); ++cir)
    {
        const vec3 &p = cir->halfedge()->vertex()->point();
        const vec2 &q = plane.to_2d(p);
        plg.push_back(to_cgal_point(q));
    }
    return plg.is_simple();
}

bool
Reconstruction::reconstruct(PointSet *pset, Map *foot_print, Map *result, LinearProgramSolver::SolverName solver_name,
                            bool update_display)
{
    if (!pset)
    {
        Logger::warn("-") << "point cloud data does not exist" << std::endl;
        return false;
    }

    MapFacetAttribute<VertexGroup::Ptr> buildings(foot_print, "buildings");

    bool success = false;
    StopWatch t;
    t.start();
    std::size_t num = 0;
    ProgressLogger progress(foot_print->size_of_facets());
    KdTreeSearch_var kdtree = new KdTreeSearch;
    kdtree->begin();

    int idx = 0;
    FOR_EACH_FACET(Map, foot_print, it)
    {
        Logger::out("-") << "processing " << ++idx << "/" << foot_print->size_of_facets() << " building..." << std::endl;

        VertexGroup::Ptr g = buildings[it];
        if (!g)
            continue;
        //ensure the footprint is manifold
        if (!is_simple_polygon(it))
        {
            ++num;
            continue;
        }

        std::vector<VertexGroup::Ptr> &roofs = g->children();
        for (std::size_t i = 0; i < roofs.size(); ++i)
        {
            VertexGroup *g = roofs[i];
        }
        unsigned int min_support = Method::number_region_growing * 0.75f;
        while (roofs.empty() && min_support > 6) // Liangliang: the number must be "> 6" to avoid infinite loops
        {
            extract_building_roof(pset, g,  Method::number_region_growing);
            min_support *= 0.75;
        }

        PointSet::Ptr roof_pset = create_roof_point_set(pset, roofs, g);
        PointSet::Ptr image_pset = create_projected_point_set(pset, roof_pset);

        if (roof_pset->num_points() < 20)
        {
            continue;
        }
        auto line_segs = compute_line_segment(image_pset, roof_pset, it);
        Map *building = reconstruct_single_building(roof_pset, line_segs, it, solver_name);

        if (building)
        {
            Geom::merge_into_source(result, building);
            // if (update_display)
            // {
            //     Tessellator::invalidate();
            //     MeshRender::invalidate();
            // }
            success = true;
        }
        roofs.clear();
        progress.next();
    }

    if (num > 0)
        Logger::warn("-") << "encountered " << num << " non-simple foot print "
                          << (num > 1 ? " polygons." : " polygon.") << std::endl;
	Logger::out("-") << "reconstruction done. Time: " << t.time_string() << std::endl;

    return success;
}

std::vector<std::vector<int>> Reconstruction::compute_height_field(PointSet *pset, Map::Facet *footprint)
{
    std::vector<vec3> original_points = pset->points();
    std::vector<K::Point_3> result;
    int num = original_points.size();
    for (std::size_t i = 0; i < num; i++)
    {
        vec3 point1 = original_points[i];
        K::Point_3 p(point1.x, point1.y, point1.z); // z stores the height of projection
        result.push_back(p);
    }
    Box3d box = pset->bbox();
    double x_max = box.x_max(), y_max = box.y_max(), x_min = box.x_min(), y_min = box.y_min();
    Delaunay_triangulation dt(result.begin(), result.end());

    // 2d vision of footprint
    Polygon2d plg;
    FacetHalfedgeCirculator fcir(footprint);
    for (; !fcir->end(); ++fcir)
    {
        vec3 q = fcir->halfedge()->vertex()->point();
        const vec2 r(q.x, q.y);
        plg.push_back(r);
    }

    //set the image resolution, a larger value for the sparser or noisy point set, usually (0.15,0.25).
    double point_density = Method::point_density;
    int in_x = (x_max - x_min) / point_density, in_y = (y_max - y_min) / point_density;
    width = ogf_max(in_x, in_y);
    height = width;
    int image_width = width, image_height = height;
    std::vector<std::vector<double>> image(image_width, std::vector<double>(image_height, -1));
    double delta_x = (x_max - x_min) / (image_width), delta_y = (y_max - y_min) / (image_height);
    double delta_res = ogf_max(delta_x, delta_y);
    image_y_min = y_min;
    image_x_min = x_min;
    image_delta_res = delta_res;
    double image_zmin = 1e20, image_zmax = 0;

    for (int j = 0; j < image_width; ++j)
    {
        for (int k = 0; k < image_width; ++k)
        {
            FT x = image_x_min + (k) * image_delta_res;
            FT y = image_y_min + (image_width - j - 1) * image_delta_res;
            Point_3d query_p(x, y, 0);
            vec2 q_p(CGAL::to_double(x), CGAL::to_double(y));
            if (Geom::point_is_in_polygon(plg, q_p))// ensure the pixel point is inside  the footprint
            {
                Face_handle fh = dt.locate(query_p);
                auto pt0 = fh->vertex(0)->point();
                auto pt1 = fh->vertex(1)->point();
                auto pt2 = fh->vertex(2)->point();
                K::Plane_3 pl(pt0, pt1, pt2);
                K::Vector_3 dir(0, 0, 1);
                K::Line_3 l(query_p, dir);
                CGAL::Object lp = CGAL::intersection(pl, l);
                if (const Point_3d *pt = CGAL::object_cast<Point_3d>(&lp))
                {
                    auto height = pt->z();
                    image[j][k] = height;
                    if (height > image_zmax)
                        image_zmax = height;
                    if (height < image_zmin)
                        image_zmin = height;
                }
            }

        }

    }

    double delta_z = 255. / (image_zmax - image_zmin);
    std::vector<std::vector<int>> im(width, std::vector<int>(height, 255));
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            auto n = image[i][j];
            if (n == -1)
            {
                im[i][j] = -1;
            } else
            {
                im[i][j] = n * delta_z - delta_z * image_zmin;
            }
        }
    }
    return im;

}

std::vector<std::vector<int>> Reconstruction::detect_height_jump(PointSet *pset,
                                                                 Map::Facet *foot_print,
                                                                 double min_height, double max_height)
{
    std::vector<std::vector<int>> im = compute_height_field(pset, foot_print);
    auto pts = pset->points();
    FacetHalfedgeCirculator cir(foot_print);
    Map::Halfedge *h = cir->halfedge();
    auto ht = h->vertex()->point().z;

    int sp = width;
    std::vector<std::vector<int >> adata(im);
    cv::Mat image3(sp, sp, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < sp; i++)
    {
        for (int j = 0; j < sp; j++)
        {
            if (adata[i][j] != -1)
            {
                float depth = adata[i][j];
                if (depth < 10)
                {
                    depth = 10;

                }
                if (depth > 245)
                {
                    depth = 245;
                }
                cv::Vec3b d(depth, depth, depth);
                image3.at<uchar>(i, j) = depth;
            }
        }
    }
    cv::Mat out1, edge;
    int k1 = (sp - 20) * 0.03;
    int kernel_size = std::max(k1, 3);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(image3, out1, 3, element);

    int low_threshold = (2.0) * 255 / (max_height - ht);
    int high_threshold = (min_height - ht) * 255 / (max_height - ht);
    high_threshold = std::max(high_threshold, 2 * low_threshold);
//	std::cout << "low: " << low_threshold << std::endl;
//	std::cout << "high: " << high_threshold << std::endl;
    cv::Canny(out1, edge, low_threshold, high_threshold);
    std::vector<std::vector<int>> point(width, std::vector<int>(width, 0));
    for (std::size_t i = 0; i < sp; i++)
    {
        for (std::size_t j = 0; j < sp; j++)
        {
            if (edge.at<uchar>(i, j) > 0)
            {
                point[i][j] = 1;
            }
        }
    }

    return point;
}


std::vector<vec3> Reconstruction::compute_line_segment(PointSet *seg_pset,
                                                       PointSet *roof_pset,
                                                       Map::Facet *footprint)
{
    std::vector<std::vector<int>> edge_point;
    std::vector<vec3> line_segments;
    if (roof_pset->points().size() > 70)
    {
        std::vector<VertexGroup::Ptr> &groups = roof_pset->groups();
        const std::vector<vec3> &roofpoints = roof_pset->points();
        std::vector<vec3> boundary_edge;
        double min_height = 1e10;
        double max_height = -1e10;

        std::vector<double> height_seq;
        for (std::size_t i = 0; i < groups.size(); ++i)
        {
            VertexGroup *g = groups[i];
            const Plane3d &plane = g->plane();
            if (std::abs(plane.normal().z) < 0.3f)
                continue;
            auto rf = g->point_set()->points();
            double avg_z = 0.;
            vec3 cen(0, 0, 0);
            std::size_t num_input = g->size();
            for (std::size_t i = 0; i < num_input; ++i)
            {
                int idx = g->at(i);
                const vec3 &p = rf[idx];
                if (p.z > max_height)
                {
                    max_height = p.z;
                }
                cen += p;

            }
            cen /= num_input;
            height_seq.push_back(cen.z);
        }

        std::sort(height_seq.begin(), height_seq.end());
        FacetHalfedgeCirculator cir(footprint);
        Map::Halfedge *h = cir->halfedge();
        auto foot_height = h->vertex()->point().z;
        for (int j = 0; j < height_seq.size(); ++j)
        {
            auto height_c = height_seq[j];
            if (height_c - foot_height > 1.0)
            {
                min_height = height_c;
                break;
            }
        }
        if (min_height > 1e5)
        {
            min_height = foot_height + 1;
        }
        edge_point = detect_height_jump(seg_pset, footprint, min_height, max_height);
        FT tolerance = 2.0;
        Otr2_edge_sim otr2(image_x_min, image_y_min, image_delta_res);
        line_segments = otr2.edge_simplify(edge_point, footprint, tolerance);
    }
    return line_segments;
}

Map *Reconstruction::reconstruct_single_building(PointSet *roof_pset,
                                                 std::vector<vec3> line_segments,
                                                 Map::Facet *footprint,
                                                 LinearProgramSolver::SolverName solver_name)
{
    // refine planes
    HypothesisGenerator hypo(roof_pset);
    hypo.refine_planes();
    // generate candidate faces
    PolyFitInfo polyfit_info;
    Map *hypothesis = hypo.generate(&polyfit_info, footprint, line_segments);
    if (!hypothesis)
        return nil;
    std::vector<Plane3d *> v = hypo.get_vertical_planes();
    // generate quality measures
    polyfit_info.generate(roof_pset, hypothesis, v, false);

    FaceSelection selector(roof_pset, hypothesis);

    selector.optimize(&polyfit_info, footprint, v, solver_name);

    extrude_boundary_to_ground(hypothesis, Geom::facet_plane(footprint), &polyfit_info);

    Geom::merge_into_source(hypothesis, footprint);
    if (hypothesis->size_of_facets() == 0)
        return 0;

    return hypothesis;
}

#define SIMPLE_EXTRUSION

void Reconstruction::extrude_boundary_to_ground(Map *model, const Plane3d &ground, PolyFitInfo *polyfit_info)
{
#ifdef SIMPLE_EXTRUSION // a planar wall might contain multiple pieces

    MapHalfedgeAttribute<bool> is_boundary(model, "is_boundary");
    std::vector<Polygon3d> polygons;
    FOR_EACH_HALFEDGE(Map, model, it)
    {
        if (is_boundary[it])
        {
            /*
            b_______a
            |       |
            |       |
            |_______|
            c       d
            */
            Polygon3d plg;
            const vec3 &a = it->vertex()->point();
            plg.push_back(a);
            const vec3 &b = it->opposite()->vertex()->point();
            plg.push_back(b);
            const vec3 &c = ground.projection(b);
            plg.push_back(c);
            const vec3 &d = ground.projection(a);
            plg.push_back(d);
            auto dis = length(c - d);
            if (dis > 1e-4)
            {
                polygons.push_back(plg);
            }
        }
    }

    if (polygons.empty())
        return;
    MapFacetAttribute<Color> color(model, "color");
    MapBuilder builder(model);
    builder.begin_surface();
    int id = 0;
    for (std::size_t i = 0; i < polygons.size(); ++i)
    {
        builder.begin_facet();
        const Polygon3d &plg = polygons[i];
        for (std::size_t j = 0; j < plg.size(); ++j)
        {
            builder.add_vertex(plg[j]);
            builder.add_vertex_to_facet(id);
            ++id;
        }
        builder.end_facet();
        color[builder.current_facet()] = random_color();
    }
    builder.end_surface();

#endif
}

Map *Reconstruction::generate_polygon(PointSet *pSet, double footprint_height,double density)
{
    //get the alpha shape of the point set
    auto alpha_boundary = AlphaShapeBoundary::apply(pSet, 1.5);
    Method::ground_height=footprint_height;
    Method::point_density=density;

    Map* new_foot = new Map;
    MapBuilder builder(new_foot);
    builder.begin_surface();
    builder.begin_facet();
    int ind=0;
    for (int i = 0; i < alpha_boundary.size(); ++i)
    {
        auto p=alpha_boundary[i];
        builder.add_vertex(vec3(p.x, p.y, footprint_height));
        builder.add_vertex_to_facet(ind);
        ++ind;
    }
    builder.end_facet();
    builder.end_surface();
    return new_foot;
}




