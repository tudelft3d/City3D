//
// Created by Jin on 19/07/2022.
//

#ifndef CITY3D_REGULARIZE_POLYGON_H
#define CITY3D_REGULARIZE_POLYGON_H
#include "cgal_types.h"
#include "../math/math_types.h"

//regularize  parallelism and orthogonality of the polygon
class RegularizePolygon {
public: std::vector<vec2> reg_ply(const std::vector<vec2>& polygon);

    std::vector<vec2> orthognal_dircs(std::vector<vec2> vector1);

    std::vector<vec2> optimize_polygon(const std::vector<vec2> &vector1, std::vector<double> vector2);
};

#endif //CITY3D_REGULARIZE_POLYGON_H
