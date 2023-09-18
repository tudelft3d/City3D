//
// Created by jinhuang on 18-9-23.
//

#ifndef CITY3D_POLYGONIZE_H
#define CITY3D_POLYGONIZE_H


#include <easy3d/core/surface_mesh.h>
#include <easy3d/util/resource.h>
#include "../model/map.h"

using  namespace easy3d;
class PolygonizeMap
{

public:
    PolygonizeMap(Map *model);
    ~PolygonizeMap();
    Map* poly();

private:
    SurfaceMesh *mesh ;
};
#endif //CITY3D_POLYGONIZE_H
