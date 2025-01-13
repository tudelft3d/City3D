#include "tessellator.h"
#include "../model/map.h"
#include "../model/map_attributes.h"
#include "../model/map_geometry.h"
#include "../basic/logger.h"

#ifdef _WIN32
#include <Windows.h>
#else
#define CALLBACK
#endif

#  if defined(__APPLE__) && defined(__MACH__)
#    include <OpenGL/glu.h>
#  else
#ifdef WIN32
#include <windows.h>
#endif
# include <GL/glu.h>
#  endif


#include <iostream>

using namespace MapTypes;


SurfaceStyle	Tessellator::surface_style_;


bool Tessellator::per_face_color_ = true;

/*
Tessellator Performance Tips:
(1) Cache the output of the tessellator in a display list or other user structure. 
    To obtain the post-tessellation vertex coordinates, tessellate the polygons 
	while in feedback mode. (See Red Book "Feedback" in Chapter 13.)
(2) Use gluTessNormal() to supply the polygon normal.
(3) Use the same tessellator object to render many polygons rather than allocate 
    a new tessellator for each one. (In a multithreaded, multiprocessor environment, 
	you may get better performance using several tessellators.)
*/


// internal data structure for a concave polygon or polygon with holes
struct ComplexPolygon {
	Polygon3d				polygon;
	Polygon2d               polygon_tex_coord;
	std::vector<Polygon3d>	holes;				// polygon without holes will be treated as concave
	std::vector<Polygon2d>	holes_tex_coord;	// polygon without holes will be treated as concave
	vec3					normal;
	Color					color;
};



static GLUtesselator*		tess_obj = 0; // the tessellation object
static std::vector<double*>	tess_points;

static GLuint				tess_display_list = 0;
static std::set<const Map*>	tess_objects;

/*  When the algorithm detects an intersection, or wishes to merge
*	features, it needs to create a new vertex.  The vertex is defined
*	as a linear combination of up to 4 existing vertices, referenced
*	by data[0..3].  The coefficients of the linear combination are
*	given by weight[0..3]; these weights always sum to 1.0.  All vertex
*	pointers are valid even when some of the weights are zero.
*	"coords" gives the location of the new vertex.     
*	The user must allocate another vertex, interpolate parameters
*	using "data" and "weights", and return the new vertex pointer in
*	"outData".  This handle is supplied during rendering callbacks.
*	For example, if the polygon lies in an arbitrary plane in 3-space,
*	and we associate a color with each vertex, the combine callback might
*	look like this:
void myCombine( GLUcoord coords[3], VERTEX *d[4], GLUcoord w[4], VERTEX **dataOut )
{
	VERTEX *new = new_vertex();

	new->x = coords[0];
	new->y = coords[1];
	new->z = coords[2];
	new->r = w[0]*d[0]->r + w[1]*d[1]->r + w[2]*d[2]->r + w[3]*d[3]->r;
	new->g = w[0]*d[0]->g + w[1]*d[1]->g + w[2]*d[2]->g + w[3]*d[3]->g;
	new->b = w[0]*d[0]->b + w[1]*d[1]->b + w[2]*d[2]->b + w[3]*d[3]->b;
	new->a = w[0]*d[0]->a + w[1]*d[1]->a + w[2]*d[2]->a + w[3]*d[3]->a;
	*dataOut = new;
}
*	If the algorithm detects an intersection, then the "combine" callback
*	must be defined, and must write a non-NULL pointer into "dataOut".
*	Otherwise the GLU_TESS_NEED_COMBINE_CALLBACK error occurs, and no
*	output is generated.
*/
void CALLBACK combineCallback(GLdouble coords[3], GLdouble *vertex_data[4],
							  GLfloat weight[4], GLdouble **dataOut)
{
	//GLdouble *vertex = (GLdouble *) malloc(6 * sizeof(GLdouble));
	GLdouble *vertex = (GLdouble *) malloc(3 * sizeof(GLdouble));
	if (vertex) {
		vertex[0] = coords[0];
		vertex[1] = coords[1];
		vertex[2] = coords[2];

		// bellow is an example that color is weighted.
		//for (int i=3; i<6; ++i) {
		//	vertex[i] = 
		//		weight[0] * vertex_data[0][i] +
		//		weight[1] * vertex_data[1][i] +
		//		weight[2] * vertex_data[2][i] +
		//		weight[3] * vertex_data[3][i];
		//}

		*dataOut = vertex;	
	} else {
		std::cout << "failed creating new vertex" << std::endl;
	}
}




// if we don't need other info (texture coord, color, etc.), we can simply register glVertex3dv)
void CALLBACK vertexCallback(GLvoid *vertex) {
	const GLdouble *ptr = (GLdouble *) vertex;
	//glColor3dv(ptr + 3);

	glVertex3dv(ptr);
}


void CALLBACK errorCallback(GLenum errorCode) {
	const GLubyte *estring = gluErrorString(errorCode);
	std::cout << "Tessellation Error: " << estring << std::endl;
	exit(0);
}


/* currently user data is not used. To use user data, 
   you may need register with the following:
GLU_TESS_BEGIN_DATA
GLU_TESS_VERTEX_DATA    
GLU_TESS_END_DATA
GLU_TESS_ERROR_DATA     
GLU_TESS_EDGE_FLAG_DATA 
GLU_TESS_COMBINE_DATA   
*/
void Tessellator::init() {
	// Create a new tessellation object 
	if (tess_obj == NULL) {
		tess_obj = gluNewTess(); 

		// Set callback functions
		gluTessCallback(tess_obj, GLU_TESS_VERTEX, (GLvoid (*)()) &vertexCallback);
		gluTessCallback(tess_obj, GLU_TESS_BEGIN, (GLvoid(*)()) &glBegin);
		gluTessCallback(tess_obj, GLU_TESS_END, (GLvoid (*)()) &glEnd);
		gluTessCallback(tess_obj, GLU_TESS_COMBINE, (GLvoid (*)()) &combineCallback);
		gluTessCallback(tess_obj, GLU_TESS_ERROR, (GLvoid (*)()) &errorCallback);
	}

	tess_points.clear();
}


void Tessellator::set_winding_rule(GLenum winding_rule) {
	if (tess_obj == NULL) {
		Logger::warn("-") << "tessellator not initialized" << std::endl;
	}

	// Set the winding rule
	gluTessProperty(tess_obj, GLU_TESS_WINDING_RULE, winding_rule); 
}


void Tessellator::render_contour(const Polygon3d& plg) {
	if (tess_obj == NULL) {
		Logger::err("-") << "tessellator not initialized" << std::endl;
		return;
	}

	gluTessBeginContour(tess_obj);

	for (std::size_t x = 0; x < plg.size(); ++x) { //loop through the vertices
		const vec3& p = plg[x];
		// always cache the vertex
		double* vts = new double[3]; // 3: (x, y, z)
		vts[0] = p.x;
		vts[1] = p.y;
		vts[2] = p.z;
		/* Specifies a vertex in the current contour for the tessellation object.
		coords contains the three-dimensional vertex coordinates, and vertex_data
		is a pointer that's sent to the callback associated with GLU_TESS_VERTEX
		or GLU_TESS_VERTEX_DATA. Typically, vertex_data contains vertex coordinates,
		surface normals, texture coordinates, color information, or whatever else the
		application may find useful. */
		gluTessVertex(tess_obj, vts, vts); //store the vertex
		tess_points.push_back(vts);
	}

	gluTessEndContour(tess_obj);
}

void Tessellator::render_contour(const Polygon3d& points, const Polygon2d& tex_coord) {
	if (tess_obj == NULL) {
		Logger::warn("-") << "tessellator not initialized" << std::endl;
	}

	gluTessBeginContour(tess_obj);

	for (unsigned int x = 0; x < points.size(); ++x) { //loop through the vertices
		const vec3& p = points[x];
		double* vts = new double[5];
 		vts[0] = p.x;
		vts[1] = p.y;
		vts[2] = p.z;

		const vec2& tc = tex_coord[x];
		vts[3] = tc.x;
		vts[4] = tc.y;
		gluTessVertex(tess_obj, vts, vts); //store the vertex
		tess_points.push_back(vts);
	}	
	
	gluTessEndContour(tess_obj);
}


void Tessellator::set_polygon_normal(const vec3& normal) {
	if (tess_obj == NULL) {
		Logger::warn("-") << "tessellator not initialized" << std::endl;
	}
	gluTessNormal(tess_obj, normal.x, normal.y, normal.z);
	glNormal3f(normal.x, normal.y, normal.z);
}


void Tessellator::begin_polygon() {
	if (tess_obj == NULL) {
		Logger::warn("-") << "tessellator not initialized" << std::endl;
	}

	gluTessBeginPolygon(tess_obj, NULL);
}


void Tessellator::end_polygon() {
	if (tess_obj == NULL) {
		Logger::warn("-") << "tessellator not initialized" << std::endl;
	}

	gluTessEndPolygon(tess_obj);
}


void Tessellator::finish() {
	if (tess_obj) {
		gluDeleteTess(tess_obj);
		tess_obj = 0;
	}

	for (unsigned int i=0; i<tess_points.size(); ++i) {
		delete [] tess_points[i];
	}
	tess_points.clear();
}

void Tessellator::clear_mesh() {
	tess_objects.clear();

	if (glIsList(tess_display_list))
		glDeleteLists(tess_display_list, 1);
}

void Tessellator::add_mesh(const Map* mesh) {
	if (mesh)
		tess_objects.insert(mesh);
}

void Tessellator::remove_mesh(const Map* mesh) {
	if (tess_objects.find(mesh) != tess_objects.end())
		tess_objects.erase(mesh); 
}

static std::vector<ComplexPolygon> collect_complex_polygons(const Map* mesh) {
	std::vector<ComplexPolygon> complex_polygons;

	if(!MapFacetNormal::is_defined(const_cast<Map*>(mesh)))
		const_cast<Map*>(mesh)->compute_facet_normals() ;
	MapFacetNormal normal(const_cast<Map*>(mesh)) ;

	MapFacetAttribute<Color> color(const_cast<Map*>(mesh), "color");
	MapFacetAttribute<bool> selected(const_cast<Map*>(mesh), "face_selected");

	FOR_EACH_FACET_CONST(Map, mesh, it) {		
		ComplexPolygon plg;		
		plg.polygon = it->to_polygon();
		plg.normal = normal[it];

		plg.color = color[it];
		if (selected[it])
			plg.color = Color(1, 0, 0);

		complex_polygons.push_back(plg);
	}

	return complex_polygons;
}

static std::vector<ComplexPolygon> collect_complex_polygons(const std::set<const Map*>& meshes) {
	std::vector<ComplexPolygon> complex_polygons;

	std::set<const Map*>::const_iterator itr = meshes.begin();
	for (; itr != meshes.end(); ++itr) {
		const std::vector<ComplexPolygon>& plgs = collect_complex_polygons(*itr);
		complex_polygons.insert(complex_polygons.end(), plgs.begin(), plgs.end());
	}
	return complex_polygons;
}


void Tessellator::invalidate() {
	if (glIsList(tess_display_list))
		glDeleteLists(tess_display_list, 1);

	std::vector<ComplexPolygon> tess_polygons = collect_complex_polygons(tess_objects);
	if (tess_polygons.empty())
		return;

	tess_display_list = glGenLists(1);
	if (tess_display_list == 0) {
		Logger::warn("-") << "failed create display list" << std::endl;
		return;
	}

	init();
	glNewList(tess_display_list, GL_COMPILE);
		glShadeModel(GL_FLAT);    
		for (unsigned int i=0; i<tess_polygons.size(); ++i) {
			const ComplexPolygon& plg_data = tess_polygons[i];
			const Polygon3d& plg = plg_data.polygon;
			//const Polygon2d& tex = plg_data.polygon_tex_coord;
			const std::vector<Polygon3d>& holes = plg_data.holes;
			//const std::vector<Polygon2d>& holes_tex_coord = plg_data.holes_tex_coord;
			const vec3& normal = plg_data.normal;
		
			if (per_face_color_) {
				const Color& c = plg_data.color;
				glColor3fv(c.data());
			}
			else 
				glColor3fv(surface_style_.color.data());

			set_polygon_normal(normal);
			if (holes.empty()) 
				set_winding_rule(GLU_TESS_WINDING_ODD);	// concave
 			else
 				set_winding_rule(GLU_TESS_WINDING_POSITIVE);	// with holes
			begin_polygon();
			//render_contour(plg, tex);
			render_contour(plg);
			for (unsigned int j = 0; j < holes.size(); ++j) {
				//render_contour(holes[j], holes_tex_coord[j]);
				render_contour(holes[j]);
			}
			end_polygon();
		}
	glEndList();
	finish();
}

void Tessellator::draw() {
	if (surface_style_.visible && tess_objects.size() > 0) {
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(0.5f, -0.0001f);

		if (glIsList(tess_display_list))
			glCallList(tess_display_list);

		glDisable(GL_POLYGON_OFFSET_FILL);
	}
}


void Tessellator::terminate() {
	clear_mesh();

	if (glIsList(tess_display_list))
		glDeleteLists(tess_display_list, 1);
}


void Tessellator::set_per_face_color(bool b) { 
	if (per_face_color_ != b) {
		per_face_color_ = b;
		invalidate();
	}
}


const SurfaceStyle& Tessellator::surface_style() {
	return surface_style_;
}

void Tessellator::set_surface_style(const SurfaceStyle& x) {
	surface_style_ = x;
	invalidate();
}
