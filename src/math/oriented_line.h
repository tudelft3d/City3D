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

#ifndef _MATH_ORIENTED_LINE_H_
#define _MATH_ORIENTED_LINE_H_


#include "math_types.h"

/**
* OrientedLine implements plucker coordinates, which enables
*  oriented lines to be compared. The comparison, implemented by the
*  side() function, is a predicate similar to the right hand rule.
* For instance, this class is used for the line-polygon intersection test.
* (see D.M.Y. Sommerville, Analytical Geometry of Three Dimensions.
*  Cambridge University Press, 1959).
*/
template <class FT> 
class GenericOrientedLine {
public:
	typedef vecng<3,FT> Point ;

	GenericOrientedLine(const Point& p, const Point& q) ;
	GenericOrientedLine() ;

	/**
	* "right hand rule" like predicate.
	*/
	static Sign side(
		const GenericOrientedLine<FT>& a, 
		const GenericOrientedLine<FT>& b 
		) ;

private:
	FT pi_[6] ;
} ;

//_________________________________________________________

template <class FT> inline
GenericOrientedLine<FT>::GenericOrientedLine(
	const Point& p, const Point& q
	) 
{
	// There are several conventions for defining plucker coordinates,
	// this one is introduced in : Marco Pellegrini, Stabbing and 
	// ray-shooting in 3-dimensional space. In Proc. 6th ACM Symposium
	// on Computational Geometry, pages 177-186, 1990.
	// I think that it is possible to have a more symmetric formulation
	// of plucker coordinates, leading to a more symmetric side() 
	// predicate, but I have no time to investigate this.
	pi_[0] = p.x*q.y - p.y*q.x ;
	pi_[1] = p.x*q.z - p.z*q.x ;
	pi_[2] = p.x - q.x ;
	pi_[3] = p.y*q.z - p.z*q.y ;
	pi_[4] = p.z - q.z ;
	pi_[5] = q.y - p.y ;
}

template <class FT> inline
GenericOrientedLine<FT>::GenericOrientedLine() {
	pi_[0] = 0 ;
	pi_[1] = 0 ;
	pi_[2] = 0 ;
	pi_[3] = 0 ;
	pi_[4] = 0 ;
	pi_[5] = 0 ;
}


template <class FT> inline
Sign GenericOrientedLine<FT>::side(
								   const GenericOrientedLine<FT>& a, 
								   const GenericOrientedLine<FT>& b
								   ) 
{
	// Note: the order might seem strange, but product between
	//  lines in plucker coordinates is a permuted cross product.
	FT cross_product =
		a.pi_[0] * b.pi_[4] +
		a.pi_[1] * b.pi_[5] +
		a.pi_[2] * b.pi_[3] +
		a.pi_[4] * b.pi_[0] +
		a.pi_[5] * b.pi_[1] +
		a.pi_[3] * b.pi_[2] ;

	return ogf_sgn(cross_product) ;
}



#endif
