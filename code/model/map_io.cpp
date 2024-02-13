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



#include "map_io.h"
#include "../basic/logger.h"
#include "../basic/file_utils.h"
#include "../model/map.h"
#include "../model/map_editor.h"
#include "../model/map_geometry.h"
#include "../basic/stop_watch.h"
#include "map_serializer.h"
#include "map_serializer_obj.h"

void join_colinear_edges(Map* mesh) {
	bool colinear_found = false;
	do {
		colinear_found = false;
		FOR_EACH_FACET_CONST(Map, mesh, it) {
			Map::Halfedge* h = it->halfedge();
			do {
				vec3 e1 = Geom::vector(h);
				vec3 e2 = Geom::vector(h->next());
				e1 = normalize(e1);
				e2 = normalize(e2);
				double dot_prod = std::abs(dot(e1, e2));
				if (dot_prod > 0.999) {
					Map::Vertex* v = h->vertex();
					MapEditor editor(mesh);
					if (editor.join_edges(v)) {
						colinear_found = true;
						break;
					}
				}
				h = h->next();
			} while (h != it->halfedge());
			if (colinear_found)
				break;
		}
	} while (colinear_found);
}

void remove_duplicated_vertices(Map* mesh) {
	bool duplicate_found = false;
	do {
		duplicate_found = false;
		FOR_EACH_EDGE(Map, mesh, it) {
			vec3 e = Geom::vector(it);
			if (length(e) < 1e-5) {
				MapEditor editor(mesh);
				if (editor.collapse_edge(it)) {
					duplicate_found = true;
					break;
				}
			}
		}
	} while (duplicate_found);
}


Map* MapIO::read(const std::string& file_name)
{
	MapSerializer_var serializer = resolve_serializer(file_name);
	if (!serializer.is_nil()) {
		Map* mesh = new Map;
		mesh->set_name(file_name);

		StopWatch w;
		Logger::out("-") << "reading file ..." << std::endl;

		if (serializer->serialize_read(file_name, mesh)) {
			Logger::out("-") << "done. " << w.seconds() << " sec." << std::endl;

			remove_duplicated_vertices(mesh);
			join_colinear_edges(mesh);

			return mesh;
		}
		else {
			delete mesh;
			Logger::err("-") << "reading file failed" << std::endl;
		}
	} 

	return nil;
}


bool MapIO::save(const std::string& file_name, const Map* mesh) 
{
	MapSerializer_var serializer = resolve_serializer(file_name);
	if (!serializer.is_nil()) {
		StopWatch w;
		Logger::out("-") << "saving file ..." << std::endl;

		if (serializer->serialize_write(file_name, mesh))  {
			Logger::out("-") << "done. " << w.seconds() << " sec." << std::endl;
			return true;
		}
		else {
			Logger::err("-") << "saving file failed" << std::endl;
			return false;
		}
	}

	return false;
}


MapSerializer* MapIO::resolve_serializer(const std::string& file_name) {
	std::string extension = FileUtils::extension(file_name) ;
	String::to_lowercase(extension);

	if(extension.length() == 0) {
		Logger::err("-") << "No extension in file name" << std::endl ;
		return nil ;
	}

	MapSerializer* serializer = nil;

	if ( extension == "obj" )
		serializer = new MapSerializer_obj();
	else if ( extension == "eobj" )
		serializer = new MapSerializer_eobj();
	else { 	
		Logger::err("-") << "unknown file format" << std::endl;
		return nil;
	}

	return serializer;
}

