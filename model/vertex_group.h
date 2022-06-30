/*
Copyright (C) 2017  Liangliang Nan
http://web.siat.ac.cn/~liangliang/ - liangliang.nan@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#ifndef _VERTEX_GROUP_H_
#define _VERTEX_GROUP_H_


#include "../basic/color.h"
#include "../math/principal_axes.h"
#include "../basic/counted.h"
#include "../basic/smart_pointer.h"

#include <vector>
#include <set>
#include <algorithm>


class PointSet;
class VertexGroup : public std::vector<unsigned int>, public Counted
{
public:
	typedef SmartPointer<VertexGroup>	Ptr;

public:
	VertexGroup(PointSet* pset = nil) 
		: point_set_(pset)
		, visible_(true)
		, highlighted_(false)
	{}

	~VertexGroup() {
		clear();
	}

	size_t nb_vertice() { return size(); }

	//////////////////////////////////////////////////////////////////////////

	PointSet* point_set() { return point_set_; }
	const PointSet* point_set() const { return point_set_; }
	void set_point_set(PointSet* pset) { point_set_ = pset; }

	const Color& color() const { return color_; }
	void set_color(const Color& c) { color_ = c; }

	void fit_plane();

	void set_plane(const Plane3d& plane) { 
		plane_ = plane; 
	}
	const Plane3d& plane() const { 
		return plane_; 
	}

	const std::vector<unsigned int>& boundary() const { return boundary_; }
	void set_boundary(const std::vector<unsigned int>& bd) { boundary_ = bd; }
	
	//////////////////////////////////////////////////////////////////////////

	VertexGroup* parent() { return parent_; }
	void set_parent(VertexGroup* g) { parent_ = g; }

	std::vector<VertexGroup::Ptr>& children();
	const std::vector<VertexGroup::Ptr>& children() const;
	void set_children(const std::vector<VertexGroup::Ptr>& chld);

	void add_child(VertexGroup* g);
	void remove_child(VertexGroup* g);

	void remove_children();
	void delete_children();

	bool is_visible() const  { return visible_; }
	void set_visible(bool b) { visible_ = b; }

	bool is_highlighted() const  { return highlighted_; }
	virtual void set_highlighted(bool b) { highlighted_ = b; }

private:
	PointSet*	point_set_;
	Plane3d		plane_;
	Color		color_;

	std::vector<unsigned int>	boundary_;
	std::vector<vec3>			facet_; 

	VertexGroup*			parent_;
	std::vector<VertexGroup::Ptr>	children_;

	bool			visible_;
	bool			highlighted_;
};


class VertexGroupCmpDecreasing
{
public:
	VertexGroupCmpDecreasing() {}

	bool operator()(const VertexGroup::Ptr g0, const VertexGroup::Ptr g1) const {
		return g0->size() > g1->size();
	}
};


class VertexGroupCmpIncreasing
{
public:
	VertexGroupCmpIncreasing() {}

	bool operator()(const VertexGroup::Ptr g0, const VertexGroup::Ptr g1) const {
		return g0->size() < g1->size();
	}
};

inline std::vector<VertexGroup::Ptr>& VertexGroup::children() {
	return children_;
}

inline const std::vector<VertexGroup::Ptr>& VertexGroup::children() const {
	return children_;
}

inline void VertexGroup::set_children(const std::vector<VertexGroup::Ptr>& chld) {
	children_ = chld;
}

inline void VertexGroup::add_child(VertexGroup* g) {
	g->set_point_set(this->point_set());
	children_.push_back(g);
}

inline void VertexGroup::remove_child(VertexGroup* g) {
	std::vector<VertexGroup::Ptr>::iterator pos = std::find(children_.begin(), children_.end(), g);
	if (pos != children_.end())
		children_.erase(pos);
}

inline void VertexGroup::remove_children() {
	children_.clear();
}

inline void VertexGroup::delete_children() {
	children_.clear();
}


#endif
