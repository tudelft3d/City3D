#pragma once


/********************************************************************
created:	 2009/07/23
created:	 23:7:2009   19:20
file base:	 prism.h
author:		 Liangliang Nan
contact:     liangliang.nan@gmail.com
affiliation: Shenzhen Institute of Advanced Technology

purpose:	 
*********************************************************************/

#include "model_common.h"
#include "../math/math_types.h"

#include <vector>
#include <string>



class Map;

class MODEL_API Prism
{
public:
	Prism(void) : height_(0.005f) {} // for rendering
	~Prism() ;

	Prism& operator = (const Prism& prism) ;

	void clear();

	size_t size() const { return base_polygon_.size(); }

	void  set_height(double h, bool bTop = true) ;
	double height() const      { return height_; }

	const vec3& direction() const       { return direction_; }
	void set_direction(const vec3& dir) { direction_ = dir;  }

	Polygon3d& base_polygon() { return base_polygon_; }
	const Polygon3d& base_polygon() const { return base_polygon_; }
	void set_base_polygon(const Polygon3d& plg) { base_polygon_ = plg; }
	
	Polygon2d& tex_coord() { return tex_coord_; }
	const Polygon2d& tex_coord() const { return tex_coord_; }
	void set_tex_coord(const Polygon2d& tc) { tex_coord_ = tc; }

	Plane3d top_plane() const ;
	Plane3d base_plane() const ;

	// for construction / modification of a prism
	void add_point(const vec3& p) ;

	Map* copy_to_mesh(bool include_ends = true) const ;

private:
	Polygon3d	base_polygon_;
	Polygon2d	tex_coord_;

	double		height_;
	vec3	direction_;
};

