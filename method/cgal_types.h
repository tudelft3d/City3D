#ifndef _CGAL_TYPES_H_
#define _CGAL_TYPES_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/bounding_box.h>
#include "../math/math_types.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT Float;
typedef Kernel::FT FT;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Vector_2 Vector_2;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Line_3 Line_3;
typedef Kernel::Plane_3 Plane_3;
typedef Kernel::Iso_cuboid_3 BoundBox;
typedef CGAL::Polygon_2<Kernel> Polygon_2;

typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2> Pwh_list_2;

inline Point_2 to_cgal_point(const vec2& p)
{
	return Point_2(p.x, p.y);
}
inline Vector_2 to_cgal_vector(const vec2& v)
{
	return Vector_2(v.x, v.y);
}

inline Point_3 to_cgal_point(const vec3& p)
{
	return Point_3(p.x, p.y, p.z);
}
inline Vector_3 to_cgal_vector(const vec3& v)
{
	return Vector_3(v.x, v.y, v.z);
}

inline vec2 to_my_point(const Point_2& p)
{
	return vec2(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
}
inline vec2 to_my_vector(const Vector_2& v)
{
	return vec2(CGAL::to_double(v.x()), CGAL::to_double(v.y()));
}

inline vec3 to_my_point(const Point_3& p)
{
	return vec3(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
}
inline vec3 to_my_vector(const Vector_3& v)
{
	return vec3(CGAL::to_double(v.x()), CGAL::to_double(v.y()), CGAL::to_double(v.z()));
}

inline Plane_3 to_cgal_plane(const Plane3d& plane)
{
	return Plane_3(to_cgal_point(plane.point()), to_cgal_vector(plane.normal()));
}

// the coordinate system is defined by (orig, base1, base2)
inline Point_2 convert_to_2d(const Point_3& orig, const Vector_3& base1, const Vector_3& base2, const Point_3& p)
{
	Vector_3 vec = p - orig;
	Float x = vec * base1;
	Float y = vec * base2;
	return Point_2(x, y);
}

inline Point_3 convert_to_3d(const Point_3& orig, const Vector_3& base1, const Vector_3& base2, const Point_2& p)
{
	return orig + base1 * p.x() + base2 * p.y();
}

#endif