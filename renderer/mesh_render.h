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

#ifndef _RENDERER_SURFACE_RENDERER_H_
#define _RENDERER_SURFACE_RENDERER_H_

#include "renderer_common.h"
#include "rendering_styles.h"
#include "../model/map_attributes.h"


class Map;

class RENDERER_API MeshRender
{
public:

	static const EdgeStyle& sharp_edge_style();
	static void set_sharp_edge_style(const EdgeStyle& x);

	static void clear_mesh();
	static void remove_mesh(const Map* mesh);
	static void add_mesh(const Map* mesh);
	static void draw();
	static void invalidate(); // rebuild the display list
	static void terminate();  // delete the display list if it exists
};



#endif
