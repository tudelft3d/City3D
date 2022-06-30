/*
*  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
*  Copyright (C) 2000-2005 INRIA - Project ALICE
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*  If you modify this software, you should include a notice giving the
*  name of the person performing the modification, the date of modification,
*  and the reason for such modification.
*
*  Contact: Bruno Levy - levy@loria.fr
*
*     Project ALICE
*     LORIA, INRIA Lorraine,
*     Campus Scientifique, BP 239
*     54506 VANDOEUVRE LES NANCY CEDEX
*     FRANCE
*
*  Note that the GNU General Public License does not permit incorporating
*  the Software into proprietary programs.
*
* As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
*     Qt, SuperLU, WildMagic and CGAL
*/


#include "map_cells.h"
#include "assert.h"

#include <algorithm>


namespace MapTypes {

	//______________________________________

	bool Vertex::is_valid() const {
		return halfedge()->vertex() == this ;
	}

	void Vertex::assert_is_valid() const {
		ogf_assert(halfedge()->vertex() == this) ;
	}

	unsigned int Vertex::degree() const {
		unsigned int result = 0 ;
		Halfedge* it = halfedge() ;
		do {
			result++ ;
			it = it->next_around_vertex() ;
		} while(it != halfedge()) ;
		return result ;
	}

	bool Vertex::is_on_border() const {
		Halfedge* it = halfedge() ;
		do {
			if(it->is_border()) {
				return true ;
			}
			it = it->next_around_vertex() ;
		} while(it != halfedge()) ;
		return false ;
	}

	bool Vertex::is_connected(const Vertex* v) const {
		Halfedge* it = halfedge() ;
		do {
			if ( it->opposite()->vertex() == v )
				return true;

			it = it->next_around_vertex();
		} while(it != halfedge()) ;

		return false;
	}

	//______________________________________

	bool Halfedge::is_valid() const {
		return (
			(opposite()->opposite() == this) &&
			(next()->prev() == this) &&
			(prev()->next() == this) 
			) ;
	}

	void Halfedge::assert_is_valid() const {
		ogf_assert(opposite()->opposite() == this) ;
		ogf_assert(next()->prev() == this) ;
		ogf_assert(prev()->next() == this) ;
	}

	//______________________________________

	Polygon3d Facet::to_polygon() const {
		Polygon3d plg;

		Halfedge* it = halfedge();
		do {
			plg.push_back(it->vertex()->point());
			it = it->next();
		} while (it != halfedge());

		return plg;
	}

	int Facet::degree() const {
		int result = 0 ;
		Halfedge* it = halfedge() ;
		do {
			result++ ;
			it = it->next() ;
		} while(it != halfedge()) ;
		return result ;
	}

	bool Facet::is_triangle() const {
		return ( halfedge()->next()->next()->next() == halfedge() ) ;
	}

	bool Facet::is_on_border() const {
		Halfedge* it = halfedge() ;
		do {
			if(it->opposite()->is_border()) {
				return true ;
			}
			it = it->next() ;
		} while(it != halfedge()) ;
		return false ;
	}

	bool Facet::is_valid() const {
		return (
			halfedge()->facet() == this &&
			degree() > 2
			) ;
	}

	void Facet::assert_is_valid() const {
		ogf_assert(halfedge()->facet() == this) ;
		ogf_assert(nb_edges() > 2) ;
	}

	void Facet::add_hole(const Facet* hole) {
		std::vector<const Facet*>::iterator pos = std::find(holes_.begin(), holes_.end(), hole);
		if (pos == holes_.end() && hole->hole_holder() != this) {
			holes_.push_back(hole);
			const_cast<Facet*>(hole)->set_hole_holder(this);
		}
		else {
			std::cout << "this facet already has the hole" << std::endl;
		}
	}

	void Facet::remove_hole(const Facet* hole) {
		if (hole->hole_holder() == this) {
			std::vector<const Facet*>::iterator pos = std::find(holes_.begin(), holes_.end(), hole);
			if (pos != holes_.end()) {
				holes_.erase(pos);
				const_cast<Facet*>(hole)->set_hole_holder(nil);
			}
		}
		else {
			std::cout << "this facet doesn't have the hole" << std::endl;
		}
	}

	void Facet::set_hole_holder(Facet* f) {
		if (hole_holder_ == f) {
			std::cout << "this facet is already the hole of the facet" << std::endl;
			return;
		}

		hole_holder_ = f;
	}

	//______________________________________

}
