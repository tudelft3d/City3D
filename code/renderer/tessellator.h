#pragma once


#include "../math/math_types.h"
#include "../renderer/rendering_styles.h"

#include <vector>
#include <string>


class Map;

// inspired by http://www.flipcode.com/archives/Polygon_Tessellation_In_OpenGL.shtml

/* usage

------- no display list, suitable for just a few polygons or in select mode ---------

FOR_EACH_FACET(Map, target, it) {
	Tessellator::init();
	const Polygon3d& plg = it->to_polygon();
	const std::vector<Polygon3d>& holes = it->holes();
	if (holes.empty()) 
		Tessellator::set_winding_rule(GLU_TESS_WINDING_ODD);		// concave
	else
		Tessellator::set_winding_rule(GLU_TESS_WINDING_POSITIVE);// with holes
	
	Tessellator::begin_polygon();
	Tessellator::render_contour(plg);
	for (unsigned int j=0; j<holes.size(); ++j)
		Tessellator::render_contour(holes[j]);
	Tessellator::end_polygon();
	Tessellator::finish();
}	

------- with display list, suitable for a large amount of complex polygons ---------

// in your data preparation
	Tessellator::add_object(mesh_0);
	Tessellator::add_object(mesh_1);
	...
	Tessellator::add_object(mesh_m);

// when you data preparation finish
	Tessellator::invalidate();

// in your draw function
	Tessellator::draw();

// in destructor of your viewer
	Tessellator::terminate();

*/

class Tessellator
{
public:
	//---------------------------------------------------------------
	// if you have only a few complex polygons, call the following 
	// functions sequentially in you draw() function.

	static void init();
	static void set_winding_rule(unsigned int winding_rule);
	static void begin_polygon();
	static void set_polygon_normal(const vec3& normal);
	static void render_contour(const Polygon3d& plg);	// without texture coords
	static void render_contour(const Polygon3d& points, const Polygon2d& tex_coord);
	static void end_polygon();
	static void finish();

	//---------------------------------------------------------------

	// if you have many complex polygons, you may need display list. 
	// first call add_object() several times, and then call invalidate() 
	// when your data is ready or if your data changes. To draw the objects,
	// just draw() in your draw function.

	static void clear_mesh();
	static void remove_mesh(const Map* mesh);
	static void add_mesh(const Map* mesh);
	static void invalidate(); // delete old display list and build new display list. 
	static void draw(); // render
	static void terminate(); // delete the display list if it exists

	static const SurfaceStyle& surface_style();
	static void set_surface_style(const SurfaceStyle& x);

	static void set_per_face_color(bool b);
	static bool per_face_color() { return per_face_color_; }

private:
	static bool		per_face_color_;

	static SurfaceStyle	surface_style_;

};
