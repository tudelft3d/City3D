#ifndef _ALGORITHM_POINT_SET_NORMALS_H_
#define _ALGORITHM_POINT_SET_NORMALS_H_


#include "../math/math_types.h"


class PointSet;

class PointSetNormals
{
public:
	static void estimate(PointSet* pset, int k = 16);
};

#endif

