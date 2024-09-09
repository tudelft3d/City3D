#ifndef _GEOMETRY_POINT_SET_GEOMETRY_H_
#define _GEOMETRY_POINT_SET_GEOMETRY_H_


#include "../math/math_types.h"



class PointSet;
class VertexGroup;


class PointSetNormalizer {
public:
	PointSetNormalizer(PointSet* pset);
	void apply(double normalized_radius = 1.0f);
	void unapply();

private:
	PointSet*	pset_;
	vec3		center_;
	double		radius_;
	double		normalized_radius_;
};


// Adds some functions related to PointSet to the Geom namespace.
namespace Geom {

	PointSet*  duplicate(const PointSet* pset);

	Box3d		bounding_box(const PointSet* pset, bool accurate = false, int samples = 200000);

	void		reverse_orientation(VertexGroup* g);

}

#endif

