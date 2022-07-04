#include "otr2_edge_simplify.h"

#include <fstream>

#include <CGAL/bounding_box.h>
#include <CGAL/linear_least_squares_fitting_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Iso_rectangle_2.h>
#include <CGAL/squared_distance_2.h> //for 2D functions
#include <boost/shared_ptr.hpp>

#include "../basic/logger.h"
#include "../model/map_circulators.h"
#include "../model/map_io.h"
#include "../model/point_set.h"

#include "dbscan.h" 
#include "cgal_types.h" 

using FT = typename K::FT;
using Indices = std::vector<std::size_t>;
using Segments = std::vector<Segment>;

FT Otr2_edge_sim::max_distance(std::vector<Point> p, Otr_2 &otr2)
{
    std::vector<Point> isolated_points;
    std::vector<Segment> segments;

    otr2.list_output(std::back_inserter(isolated_points), std::back_inserter(segments));
    FT current_dist = 0;
    // segments
    std::vector<Segment>::iterator sit;
    for (sit = segments.begin(); sit != segments.end(); sit++)
    {
        FT dist = 1e10;
        for (int i = 0; i < p.size(); i++)
        {
            Point t = p[i];
            FT d1 = CGAL::squared_distance(t, *sit);
            dist = std::min(dist, d1);
        }
        current_dist = std::max(current_dist, dist);
    }
    return current_dist;
}

std::vector<vec3> Otr2_edge_sim::edge_simplify(std::vector<std::vector<int>> edge_point,
                                               Map::Facet *foot_print,
                                               double distance)
{

    std::vector<Point> points;
    int sp = edge_point.size();
    for (std::size_t i = 0; i < sp; i++)
    {
        for (std::size_t j = 0; j < sp; j++)
        {
            if (edge_point[i][j] > 0)
            {
                FT a = i, b = j;
                Point np(a, b);
                points.push_back(np);
            }
        }
    }

    Otr_2 otr2(points);
    FT m = 0;
    FT sigma_d = 0.25;
    FT tolerance = 2.0;
    do
    {
        int num1 = otr2.number_of_all_edges();
        //in each step , total cost less than this tolrrance
        otr2.run(1, 2.0);
        int num2 = otr2.number_of_all_edges();
        m = max_distance(points, otr2);
        if (num1 == num2)// no more edge or vertex can be removed
            break;
    } while (m < sigma_d); //ensure hausdorff distance less than this threshold
    if (otr2.number_of_all_edges() > 80)
    {
        FT new_sigma_d = 1.5 * sigma_d;
        FT new_tolerance = 1.5 * tolerance;
        do
        {
            int num1 = otr2.number_of_all_edges();
            //in each step , total cost less than this tolrrance
            otr2.run(1, new_tolerance);
            int num2 = otr2.number_of_all_edges();
            m = max_distance(points, otr2);
            if (num1 == num2)// no more edge or vertex can be removed
                break;
        } while (m < new_sigma_d); //ensure hausdorff distance less than this threshold
    }
    std::vector<vec3> line_segments;
    line_segments = cluster_lines(otr2, foot_print, sp);
    return line_segments;

}

double compute_avg_length(Map::Facet *foot_print)
{
    double len = 0;
    FacetHalfedgeCirculator fcir(foot_print);
    for (; !fcir->end(); ++fcir)
    {
        vec3 q = fcir->halfedge()->vertex()->point();
        vec3 q1 = fcir->halfedge()->opposite()->vertex()->point();
        double dis2 = length(q - q1);
        len += dis2;
    }
    len /= foot_print->nb_edges();
    return 0.5*len;
}

std::vector<vec2> compute_principle_directions(Map::Facet *foot_print)
{
    std::vector<vec2> dirs;
    auto len = compute_avg_length(foot_print);
    FacetHalfedgeCirculator fcir(foot_print);
    for (; !fcir->end(); ++fcir)
    {
        vec3 q = fcir->halfedge()->vertex()->point();
        vec2 r(q[0], q[1]);
        vec3 q1 = fcir->halfedge()->opposite()->vertex()->point();
        vec2 r1(q1[0], q1[1]);
        vec2 rd = r - r1;
        if (length(rd) > len)
            dirs.push_back(normalize(rd));
    }
    return dirs;
}

float dist_fun(vec4 a, vec4 b)
{
    vec2 point_a0(a[0], a[1]), point_a1(a[2], a[3]),
            point_b0(b[0], b[1]), point_b1(b[2], b[3]);
    Point p_a0(a[0], a[1]), p_a1(a[2], a[3]),
            p_b0(b[0], b[1]), p_b1(b[2], b[3]);
    Segment sa(p_a0, p_a1), sb(p_b0, p_b1);
    Line la(p_a0, p_a1), lb(p_b0, p_b1);
    double normal_deviation, dist;
    vec2 dir_a = normalize(point_a0 - point_a1), dir_b = normalize(point_b0 - point_b1);
    normal_deviation = std::acos(std::abs(dot(dir_a, dir_b)));
    auto d1 = std::sqrt(CGAL::squared_distance(sa, lb));
    auto d2 = std::sqrt(CGAL::squared_distance(sb, la));
    dist = std::max(d1, d2);
    return 3.5 * normal_deviation + dist;

}

std::vector<vec3> Otr2_edge_sim::cluster_lines(Otr_2 &otr2, Map::Facet *foot_print, int &width)
{
    std::vector<Point> points;
    std::vector<std::size_t> isolated_vertices;
    std::vector<std::pair<std::size_t, std::size_t> > edges;
    otr2.indexed_output(
            std::back_inserter(points),
            std::back_inserter(isolated_vertices),
            std::back_inserter(edges));
    std::vector<std::pair<std::size_t, std::size_t> > back_edges(edges);
    // points

    FT x_min(image_x_min);
    FT y_min(image_y_min);
    FT interval(image_delta_res);
    std::vector<Point> new_point;
    for (std::size_t i = 0; i < points.size(); i++)
    {
        FT row = points[i][0];
        FT col = points[i][1];
        FT x = x_min + (col) * interval;
        FT y = y_min + (width - row - 1) * interval;
        Point np(x, y);
        new_point.push_back(np);
    }
    // edges
    std::vector<std::pair<std::size_t, std::size_t> >::iterator eit;
    //delete the wrong line segments
    auto main_dirs = compute_principle_directions(foot_print);

    //cluster the lines based on the orientation and distance using dbscan;
    DBSCAN<vec4, float> dbscan;
    std::vector<vec4> point_data;
    std::vector<Point> input_cluster_point, output_points;
    std::vector<std::pair<std::size_t, std::size_t> > input_edges, output_edges;
    for (int i = 0; i < edges.size(); i++)

        std::vector<std::pair<std::size_t, std::size_t> > edge_pairs;
    // points
    for (int i = 0; i < edges.size(); i++)
    {
        std::size_t index_1 = edges[i].first;
        std::size_t index_2 = edges[i].second;
        //new_points (2*i,2*i+1) -> segments(i)
        input_cluster_point.push_back(new_point[index_1]);
        input_cluster_point.push_back(new_point[index_2]);
        input_edges.push_back(std::make_pair(2 * i, 2 * i + 1));
        point_data.push_back(vec4(new_point[index_1][0],
                                  new_point[index_1][1],
                                  new_point[index_2][0],
                                  new_point[index_2][1]));
    }


    dbscan.Run(&point_data, 4, 1.2f, 1, dist_fun);
    auto noise = dbscan.Noise;
    auto clusters = dbscan.Clusters;

    for (int ind = 0; ind < clusters.size(); ++ind)
    {
        auto cls = clusters[ind];
        //compute the average line segments and generate new one
        std::vector<vec2> dirs;
        std::vector<double> lens;
        vec2 cen;
        std::vector<Point> pts;
        double cls_xmin = 1e10, cls_xmax = -1e10, cls_ymin = 1e10, cls_ymax = -1e10;

        for (int j = 0; j < cls.size(); ++j)
        {
            auto idx = cls[j];
            auto seg = point_data[idx];
            vec2 dir(seg[2] - seg[0], seg[3] - seg[1]);
            dirs.push_back(normalize(dir));
            lens.push_back(length(dir));
            vec2 tmp_cen(seg[2] + seg[0], seg[3] + seg[1]);
            cen += tmp_cen;
            pts.push_back(Point(seg[0], seg[1]));
            pts.push_back(Point(seg[2], seg[3]));
        }
        for (int i = 0; i < pts.size(); ++i)
        {
            auto p = pts[i];
            cls_xmax = std::max(cls_xmax, p[0]);
            cls_ymax = std::max(cls_ymax, p[1]);
            cls_xmin = std::min(cls_xmin, p[0]);
            cls_ymin = std::min(cls_ymin, p[1]);
        }
        cen /= (2 * cls.size());
        vec2 avg_dir;
        double total_len;
        for (int i = 0; i < dirs.size(); ++i)
        {
            auto tmp_dir = dirs[i];
            if (dot(tmp_dir, dirs[0]) < 0)
            {
                tmp_dir = -tmp_dir;
            }
            avg_dir += tmp_dir * lens[i];
            total_len += lens[i];
        }
        avg_dir /= total_len;
        Line cen_line(Point(cen.x, cen.y), Vector(avg_dir.x, avg_dir.y));
        CGAL::Iso_rectangle_2<K> abox(Point(cls_xmin, cls_ymin), Point(cls_xmax, cls_ymax));
        auto result = CGAL::intersection(abox, cen_line);
        if (const Segment *s = boost::get<Segment>(&*result))
        {
            output_points.push_back(s->point(0));
            output_points.push_back(s->point(1));
            output_edges.push_back(std::make_pair(2 * ind, 2 * ind + 1));
        }
    }
    auto cur_size = 2 * output_edges.size();
    for (int i = 0; i < noise.size(); ++i)
    {
        auto ind = noise[i];
        auto seg = point_data[ind];
        output_points.push_back(Point(seg[0], seg[1]));
        output_points.push_back(Point(seg[2], seg[3]));
        output_edges.push_back(std::make_pair(2 * i + cur_size, 2 * i + 1 + cur_size));
    }

    auto pts = output_points;
    auto edge_pairs = output_edges;

    //rotate the direction of the line segments to make them strictly align with  the footprint edge
    std::vector<Point> rotated_points;
    std::vector<std::pair<std::size_t, std::size_t> > rotated_edges;

    for (eit = edge_pairs.begin(); eit != edge_pairs.end(); eit++)
    {
        std::size_t index_1 = eit->first;
        std::size_t index_2 = eit->second;
        vec2 p_1(pts[index_1][0], pts[index_1][1]);
        vec2 p_2(pts[index_2][0], pts[index_2][1]);
        vec2 p_d = p_1 - p_2;
        double length_pd = length(p_d);
        vec2 r_direction;
        vec2 p_mid = (p_2 + p_1) * 0.5;
        double max_cos = 0;
        FacetHalfedgeCirculator fcir(foot_print);
        for (auto dir: main_dirs)
        {
            double angel = std::abs(Geom::cos_angle(dir, p_d));
            if (angel > max_cos)
            {
                max_cos = angel;
                r_direction = dir;
            }
        }

        double length_rd = length(r_direction);

        p_1 = p_mid + 0.5 * length_pd * r_direction / length_rd;
        p_2 = p_mid - 0.5 * length_pd * r_direction / length_rd;
        rotated_points.push_back(to_cgal_point(p_1));
        rotated_points.push_back(to_cgal_point(p_2));
        rotated_edges.push_back(std::make_pair(rotated_points.size() - 1, rotated_points.size() - 2));

    }
    new_point = rotated_points;
    edges = rotated_edges;

    std::vector<vec3> temp_p;
    for (int i = 0; i < new_point.size(); i++)
    {
        vec3 tp(new_point[i][0], new_point[i][1], 0);
        temp_p.push_back(tp);
    }
    for (eit = edges.begin(); eit != edges.end(); eit++)
    {
        vec3 tp1(2, eit->first, eit->second);
        temp_p.push_back(tp1);
    }
    return temp_p;
}








