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

#include "mesh_render.h"
#include "opengl_info.h"
#include "../model/map_geometry.h"
#include "../basic/logger.h"


static EdgeStyle			g_sharp_edge_style;

static GLuint				g_display_list = 0;
static std::set<const Map*>	g_objects;



void MeshRender::clear_mesh() {
	g_objects.clear();

	if (glIsList(g_display_list))
		glDeleteLists(g_display_list, 1);
}

void MeshRender::add_mesh(const Map* mesh) {
	if (mesh)
		g_objects.insert(mesh);
}

void MeshRender::remove_mesh(const Map* mesh) {
	if (g_objects.find(mesh) != g_objects.end())
		g_objects.erase(mesh);
}


const EdgeStyle& MeshRender::sharp_edge_style() {
	return g_sharp_edge_style;
}

void MeshRender::set_sharp_edge_style(const EdgeStyle& x) {
	g_sharp_edge_style = x;
}

void MeshRender::terminate() {
	clear_mesh();

	if (glIsList(g_display_list))
		glDeleteLists(g_display_list, 1);
}

void MeshRender::invalidate() {
	if (glIsList(g_display_list))
		glDeleteLists(g_display_list, 1);

	g_display_list = glGenLists(1);
	if (g_display_list == 0) {
		Logger::warn("-") << "failed create display list" << std::endl;
		return;
	}

	glNewList(g_display_list, GL_COMPILE);
	for (std::set<const Map*>::const_iterator it = g_objects.begin(); it != g_objects.end(); ++it) {
		const Map* mesh = *it;
		glBegin(GL_LINES);
		FOR_EACH_EDGE_CONST(Map, mesh, it) {
			bool need_draw = it->is_border_edge();
			if (!it->is_border_edge()) {
				const vec3& n1 = Geom::facet_normal(it->facet());
				const vec3& n2 = Geom::facet_normal(it->opposite()->facet());
				if (std::abs(dot(n1, n2)) > 0.99)
					need_draw = false;
			}
			if (need_draw) {
				const vec3& p = it->vertex()->point();
				const vec3& q = it->opposite()->vertex()->point();
				glVertex3dv(p.data());
				glVertex3dv(q.data());
			}
		}
		glEnd();
	}

	glEndList();
}


void MeshRender::draw() {
	if (g_sharp_edge_style.visible && g_objects.size() > 0) {
		glColor4fv(g_sharp_edge_style.color.data());
		glDisable(GL_LIGHTING);
		glLineWidth(g_sharp_edge_style.width);

		if (glIsList(g_display_list))
			glCallList(g_display_list);

		glEnable(GL_LIGHTING);
	}
}