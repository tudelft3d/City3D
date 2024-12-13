//
// Created by Jin on 02/08/2022.
//

#ifndef CITY3D_INTERSECTION_AREA_H
#define CITY3D_INTERSECTION_AREA_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <iostream>

struct FaceInfo2 {
    FaceInfo2()
    {}

    int nesting_level;

    bool in_domain()
    {
        return nesting_level % 2 == 1;
    }
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K> Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> TDS;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
typedef CDT::Triangle Triangle;
typedef CDT::Point Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CDT::Face_handle face_handle;


void
mark_domains(CDT &ct,
             face_handle start,
             int index,
             std::list<CDT::Edge> &border)
{
    if (start->info().nesting_level != -1)
    {
        return;
    }
    std::list<face_handle> queue;
    queue.push_back(start);

    while (!queue.empty())
    {
        face_handle fh = queue.front();
        queue.pop_front();
        if (fh->info().nesting_level == -1)
        {
            fh->info().nesting_level = index;
            for (int i = 0; i < 3; i++)
            {
                CDT::Edge e(fh, i);
                face_handle n = fh->neighbor(i);
                if (n->info().nesting_level == -1)
                {
                    if (ct.is_constrained(e)) border.push_back(e);
                    else queue.push_back(n);
                }
            }
        }
    }
}

void
mark_domains(CDT &cdt)
{
    for (CDT::Face_handle f: cdt.all_face_handles())
    {
        f->info().nesting_level = -1;
    }

    std::list<CDT::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);
    while (!border.empty())
    {
        CDT::Edge e = border.front();
        border.pop_front();
        face_handle n = e.first->neighbor(e.second);
        if (n->info().nesting_level == -1)
        {
            mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
        }
    }
}


double intersection_area(std::vector<vec2> s, std::vector<vec2> t)
{
    //construct two non-intersecting nested polygons
    Polygon_2 polygon1, polygon2;
    for (int i = 0; i < s.size(); i++)
    {
        polygon1.push_back(Point_2(s[i].x, s[i].y));
    }
    for (int i = 0; i < t.size(); i++)
    {
        polygon2.push_back(Point_2(t[i][0], t[i][1]));
    }
    if (!polygon2.is_simple())
    {return 0;}
    //Insert the polygons into a constrained triangulation
    CDT cdt, cdt1;
    cdt.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(), true);
    cdt1.insert_constraint(polygon2.vertices_begin(), polygon2.vertices_end(), true);

    //Mark facets that are inside the domain bounded by the polygon
    mark_domains(cdt);
    mark_domains(cdt1);

    double area_inter = 0;
    int icount = 0;
    for (auto i: cdt.finite_face_handles())
    {
        if (i->info().in_domain())
        {
            auto tr_i = cdt.triangle(i);

            for (auto j: cdt1.finite_face_handles())
            {
                if (j->info().in_domain())
                {
                    icount++;
                    auto tr_j = cdt1.triangle(j);
                    CGAL::Object object = CGAL::intersection(tr_i, tr_j);

                    if (const Triangle *ptr = CGAL::object_cast<Triangle>(&object))
                        area_inter += CGAL::abs((*ptr).area());
                    else if (const std::vector<Point_2> *ptr = CGAL::object_cast<std::vector<Point_2> >(&object))
                    {
                        std::vector<Point_2> v = (*ptr);
                        if (v.size() < 3)
                        { std::cout << "nononono................." << std::endl; }

                        for (size_t k = 1; k < v.size() - 1; k++)
                        {
                            Triangle t_k(v[0], v[k], v[k + 1]);
                            area_inter += CGAL::abs((t_k).area());
                        }

                    }
                }
            }
        }
    }
    return area_inter;

}



#endif //CITY3D_INTERSECTION_AREA_H
