//
// Created by jinhuang on 18-9-23.
//
#include <easy3d/util/initializer.h>
#include <easy3d/algo/surface_mesh_polygonization.h>
#include <easy3d/algo/surface_mesh_triangulation.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/util/resource.h>
#include <easy3d/algo_ext/surfacer.h>
#include "Polygonize.h"
#include "easy3d/core/surface_mesh_builder.h"
#include "easy3d/fileio/surface_mesh_io.h"


using namespace easy3d;


PolygonizeMap::PolygonizeMap(Map *model)
{
     mesh = SurfaceMeshIO::load("./ini.obj");
};

PolygonizeMap::~PolygonizeMap()
{
};

Map* PolygonizeMap::poly()
{
    // STEP 1: Repairing as implemented in Easy3D client
    Surfacer::repair_polygon_soup(mesh);

    // STEP 2: Polygonization as implemented in Easy3D client

    // stitch first: to encourage large polygons
    Surfacer::stitch_borders(mesh);
    Surfacer::merge_reversible_connected_components(mesh);

    // polygonization
    SurfaceMeshPolygonization polygonizer;
    polygonizer.apply(mesh);

    // stitch again
    Surfacer::stitch_borders(mesh);
    Surfacer::merge_reversible_connected_components(mesh);
    SurfaceMeshIO::save("./2.obj", mesh);
};





