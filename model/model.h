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

#ifndef _MODEL_H_
#define _MODEL_H_


#include "model_common.h"
#include "../basic/basic_types.h"
#include "../math/math_types.h"


class MODEL_API Model
{
public:
	Model() : offset_(0, 0, 0) {}
	~Model() {}

	const std::string& name() const { return name_; }
	void set_name(const std::string& n) { name_ = n; }

	// the offset occurred during file reading
	void set_offset(const vec3& os) { offset_ = os; }
	const vec3& offset() const { return offset_; }

private:
	vec3 offset_;
	std::string	name_;
};


#endif // _POINT_SET_H_
