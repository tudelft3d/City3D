#include <fstream>
#include "point_set_region_growing.h"
#include "../model/point_set.h"
#include "../model/point_set_geometry.h"
#include "../basic/logger.h"
#include "../math/math_types.h"
#include <list>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection/Region_growing.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Plane_3 Plane_3;
// Point with normal, and plane index.
typedef boost::tuple<Point, Vector, int> PNI;
typedef std::vector<PNI> Point_vector;
typedef CGAL::Nth_of_tuple_property_map<0, PNI> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNI> Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNI> Plane_index_map;
typedef CGAL::Shape_detection::Point_set::
Sphere_neighbor_query<Kernel, Point_vector, Point_map> Neighbor_query;
typedef CGAL::Shape_detection::Point_set::
Least_squares_plane_fit_region<Kernel, Point_vector, Point_map, Normal_map> Region_type;
typedef CGAL::Shape_detection::
Region_growing<Point_vector, Neighbor_query, Region_type> Region_growing;

std::vector<VertexGroup::Ptr> do_detect(
	PointSet* pset,
	std::vector<std::size_t> p_index,
	Point_vector points,
	std::vector<Point> pots,
	std::vector<Vector> nos,
	std::vector<unsigned int>& unassigned_points,
	unsigned int min_support,
	double dist_thresh,
	double bitmap_reso,
	double normal_thresh
)
{

	std::vector<VertexGroup::Ptr> results;
	CGAL::parameters::point_map(pots);
	CGAL::parameters::normal_map(nos);

	const FT search_sphere_radius = FT(100) / FT(100);
	const FT max_distance_to_plane = FT(50) / FT(100);
	const FT max_accepted_angle = FT(30);
	const std::size_t min_region_size = min_support;
	// Create instances of the classes Neighbor_query and Region_type.
	Neighbor_query neighbor_query(
		points,
		search_sphere_radius);
	Region_type region_type(points,
		max_distance_to_plane, max_accepted_angle, min_region_size);
	// Create an instance of the region growing class.
	Region_growing region_growing(
		points, neighbor_query, region_type);
	std::vector<std::vector<std::size_t> > regions;
	region_growing.detect(std::back_inserter(regions));
	for (std::size_t i = 0; i < regions.size(); ++i)
	{
		std::list<int> vts;
		std::list<Point> pts;
		for (const std::size_t idx : regions[i])
		{
			pts.push_back(points[idx].get<0>());
			auto index = p_index[idx];
			vts.push_back(index);
		}
		Plane_3 supporting_plane_;
		CGAL::linear_least_squares_fitting_3(pts.begin(), pts.end(), supporting_plane_, CGAL::Dimension_tag<0>());
		Plane3d plane1(supporting_plane_.a(), supporting_plane_.b(), supporting_plane_.c(), supporting_plane_.d());
		VertexGroup* group = new VertexGroup;
		group->set_point_set(pset);
		group->insert(group->end(), vts.begin(), vts.end());
		group->set_plane(plane1);
		results.push_back(group);
	}
	std::vector<std::size_t> unassigned_items;
	region_growing.unassigned_items(std::back_inserter(unassigned_items));
	unassigned_points.clear();
	for (auto p_in:unassigned_items)
	{
		unsigned int m_un = p_index[p_in];
		unassigned_points.push_back(m_un);
	}
	auto remaining = unassigned_items.size();
//	Logger::out() << results.size() << " primitives extracted. " << remaining << " points remained." << std::endl;
	return results;
}

std::vector<VertexGroup::Ptr> Region_Growing_Dectetor::detect(
	PointSet* pset,
	unsigned int min_support,
	double dist_thresh,
	double bitmap_reso,
	double normal_thresh)
{
	std::vector<VertexGroup::Ptr> results;
	Logger::out() << "entire point set" << std::endl;

	if (pset == nil)
	{
		Logger::out() << "no data exists" << std::endl;
		return results;
	}
	if (pset->num_points() < 3)
	{
		Logger::out() << "point set has less than 3 points" << std::endl;
		return results;
	}

	if (!pset->has_normals())
	{
		Logger::out() << "Region Growing Detector requires point cloud normals" << std::endl;
		return results;
	}
	Point_vector pwn;
	std::vector<std::size_t> p_index;
	p_index.resize(pset->num_points());
	std::vector<Point> pts;
	std::vector<Vector> nos;
	pts.resize(pset->num_points());
	nos.resize(pset->num_points());
	pwn.resize(pset->num_points());
	const std::vector<vec3>& normals = pset->normals();
	const std::vector<vec3>& points = pset->points();
#pragma omp parallel for
	for (int i = 0; i < points.size(); ++i)
	{
		const vec3& p = points[i];
		const vec3& n = normals[i];
		pwn[i] = boost::make_tuple(Point(p.x, p.y, p.z), Vector(n.x, n.y, n.z), -1);
		pts[i] = Point(p.x, p.y, p.z);
		nos[i] = Vector(n.x, n.y, n.z);
		p_index[i] = i;
	}
	return do_detect(pset,
		p_index,
		pwn,
		pts,
		nos,
		unassigned_points,
		min_support,
		dist_thresh,
		bitmap_reso,
		normal_thresh);

}

std::vector<VertexGroup::Ptr> Region_Growing_Dectetor::detect(
	PointSet* pset,
	const std::vector<unsigned int>& vertitces,
	unsigned int min_support,    // the minimal number of points required for a primitive
	double dist_thresh,    // relative to the bounding box width. NOTE: Internally the distance threshold is taken as 3 * distance_threshold!!!
	double bitmap_reso,    // relative to the bounding box width. NOTE: This threshold is NOT multiplied internally!
	double normal_thresh    // the cos of the maximal normal deviation

)
{
	std::vector<VertexGroup::Ptr> results;

	if (pset == nil)
	{
		Logger::out() << "no data exists" << std::endl;
		return results;
	}

	if (vertitces.size() < 3)
	{
		//Logger::out() << "input has less than 3 points" << std::endl;
		return results;
	}

	if (!pset->has_normals())
	{
		Logger::out() << "Region Growing Detector requires point cloud normals" << std::endl;
		return results;
	}
	// prepare the data
	Point_vector pwn;
	std::vector<Point> pts;
	std::vector<Vector> nos;
	pwn.resize(vertitces.size());
	pts.resize(vertitces.size());
	nos.resize(vertitces.size());
	std::vector<std::size_t> p_index;
	p_index.resize(vertitces.size());
	const std::vector<vec3>& normals = pset->normals();
	const std::vector<vec3>& points = pset->points();
#pragma omp parallel for
	for (int index = 0; index < vertitces.size(); ++index)
	{
		std::size_t idx = vertitces[index];
		const vec3& p = points[idx];
		const vec3& n = normals[idx];
		pwn[index] = boost::make_tuple(Point(p.x, p.y, p.z), Vector(n.x, n.y, n.z), -1);
		pts[index] = Point(p.x, p.y, p.z);
		nos[index] = Vector(n.x, n.y, n.z);
		p_index[index] = idx;
	}

	return do_detect(pset,
		p_index,
		pwn,
		pts,
		nos,
		unassigned_points,
		min_support,
		dist_thresh,
		bitmap_reso,
		normal_thresh);

}



