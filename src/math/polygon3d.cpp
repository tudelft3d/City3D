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


#include "polygon3d.h"
#include "polygon2d.h"
#include "plane.h"
#include "../math/third_party/predicates.h"

namespace Geom {

	void barycentric_coords(
		const Polygon3d& P, const vec3& p, std::vector<double>& bary
		) {
		bary.clear();
		for (std::size_t i = 0; i<P.size(); i++) { bary.push_back(0); }

		bool degenerate = false;
		double epsilon = 1.0e-5f;
		double total = 0.0f;
		std::size_t size = P.size();

		for (std::size_t i = 0; i < size; i++)  {
			std::size_t j = i + 1;
			if (j >= size) { j = 0; }
			std::size_t k = j + 1;
			if (k >= size) { k = 0; }

			const vec3& pA = P[i];
			const vec3& pB = P[j];
			const vec3& pC = P[k];

			vec3 vectBA = pA - pB;
			vec3 vectBP = p - pB;
			vec3 vectBC = pC - pB;
			double dotABP = dot(vectBP, vectBA);
			double crossABP = length(cross(vectBP, vectBA));
			double dotPBC = dot(vectBC, vectBP);
			double crossPBC = length(cross(vectBC, vectBP));

			degenerate |= crossABP < dotABP*epsilon;
			degenerate |= crossPBC < dotPBC*epsilon;
			degenerate |= length2(vectBP) < epsilon*epsilon;

			// cotangent of angles
			double cotABP = dotABP / crossABP;
			double cotPBC = dotPBC / crossPBC;
			double factor = (cotABP + cotPBC) / length2(vectBP);
			bary[j] = factor;
			total += factor;
		}

		if (false && degenerate) {
			//degenerate case, we will have to resort to another formula
			// in this case the computation is slower - n^2 instead of n
			total = 0.0;
			for (unsigned int i = 0; i < size; i++) {
				unsigned int j = i + 1;
				if (j >= size) { j = 0; }
				unsigned int k = j + 1;
				if (k >= size) { k = 0; }

				const vec3& pA = P[i];
				const vec3& pB = P[j];
				const vec3& pC = P[k];

				double factor = length(
					cross(pA, pB) +
					cross(pB, pC) +
					cross(pC, pA)
					);
				for (unsigned int l = k; l != i; l++) {
					if (l >= size) { l = 0; }
					unsigned int m = l + 1;
					if (m >= size) { m = 0; }

					const vec3& pD = P[l];
					const vec3& pE = P[m];
					double area = length(
						cross(pD, pE) +
						cross(pE, p) +
						cross(p, pD)
						);
					factor *= area;
				}

				// there are actually 2 factors 0.5 to retrieve the real area,
				// but they are transparently taken care of in the normalization
				bary[j] = factor;
				total += factor;
			}
		}

		// normalize the coordinates and we are done
		for (unsigned int n = 0; n < size; n++)  {
			bary[n] = bary[n] / total;
		}
	}

	/**
	* Precondition: the segment [s, t] intersects the plane (q1,q2,q3).
	* If both s and t are in the plane, returns the mid-point.
	*/
	vec3 intersect_segment_with_plane(
		const vec3& s, const vec3& t,
		const vec3& q1, const vec3& q2, const vec3& q3
		) {
		Plane3d P(q1, q2, q3);
		double h = fabs(P.value(s));
		double l = fabs(P.value(t));
		double hl = h + l;
		if (hl > 0) {
			return vec3(
				(l / hl) * s.x + (h / hl) * t.x,
				(l / hl) * s.y + (h / hl) * t.y,
				(l / hl) * s.z + (h / hl) * t.z
				);
		}
		return vec3(
			0.5f*(s.x + t.x),
			0.5f*(s.y + t.y),
			0.5f*(s.z + t.z)
			);
	}

}


