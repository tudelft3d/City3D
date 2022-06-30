#ifndef _ALGORITHM_POINT_SET_NORMALS_H_
#define _ALGORITHM_POINT_SET_NORMALS_H_

#include "method_common.h"
#include "../math/math_types.h"


class PointSet;

class METHOD_API PointSetNormals
{
public:
	static void estimate(PointSet* pset, int k = 16);
};

#endif

