/*
Copyright (C) 2017  Liangliang Nan
https://3d.bk.tudelft.nl/liangliang/ - liangliang.nan@gmail.com

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

#ifndef _MATH_COMPARE_H_
#define _MATH_COMPARE_H_


#include "math_types.h"


namespace Compare {

	//////////////////////////////////////////////////////////////////////////

	template<class T>
	class ElementAndValue
	{
	public:
		ElementAndValue(const T& e, double v = 0)
			: element_(e)
			, value_(v)
		{
		}

		ElementAndValue(const ElementAndValue& ev)
			: element_(ev.element_)
			, value_(ev.value_)
		{
		}		

		ElementAndValue& operator = (const ElementAndValue<T>& ev) {
			element_ = ev.element_;
			value_ = ev.value_;
			return *this;
		}

		T		  element_;
		double    value_;
	};


	template<class T>
	class ElementAndValueLess {
	public:
		bool operator()(const ElementAndValue<T>& first, const ElementAndValue<T>& second) const {
			if(first.value_ < second.value_)
				return true;
			else
				return false;
		}
	}; 

	template<class T>
	class ElementAndValueGreater {
	public:
		bool operator()(const ElementAndValue<T>& first, const ElementAndValue<T>& second) const {
			if(first.value_ > second.value_)
				return true;
			else
				return false;
		}
	}; 


	//////////////////////////////////////////////////////////////////////////

	class IndicesAndScore {
	public:
		IndicesAndScore(unsigned int left_idx, unsigned int right_idx, unsigned int s) 
			: left_index_(left_idx)
			, right_index_(right_idx)
			, score_(s)
		{}

		IndicesAndScore(const IndicesAndScore& is) 
			: left_index_(is.left_index_)
			, right_index_(is.right_index_)
			, score_(is.score_)
		{}

		unsigned int left_index_;
		unsigned int right_index_;
		unsigned int score_;
	};


	class IndicesAndScoreGreater {
	public:
		bool operator()(const IndicesAndScore& first, const IndicesAndScore& second) const {
			if(first.score_ > second.score_)
				return true;
			else
				return false;
		}
	}; 


	//////////////////////////////////////////////////////////////////////////

	class IndexAndValue {
	public:
		IndexAndValue(unsigned int idx, const vec3& pos, double v) : index(idx), value(v) {}
		unsigned int index;
		vec3      position;
		double       value;
	};


	class IndexAndValueGreater {
	public:
		bool operator()(const IndexAndValue& first, const IndexAndValue& second) const {
			if(first.value > second.value)
				return true;
			else
				return false;
		}
	}; 
}


#endif
