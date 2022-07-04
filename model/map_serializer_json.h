
#ifndef _FILEIO_MESH_SERIALIZER_JSON_H_
#define _FILEIO_MESH_SERIALIZER_JSON_H_


#include "map.h"
#include "map_builder.h"

// read Geojson format files using the single header file JSON library:
// https://github.com/nlohmann/json
// I store 2D polygons as a 3D mesh (all z-coordinates are set to 0) consisting of
// a set of individual faces.

class MapSerializer_json
{
public:
	MapSerializer_json();

	// if use_provided_offset == true, apply 'offset' on the model; otherwise the default offset will based on the first point in the model.
	Map* read(const std::string& file_name, bool use_provided_offset = false, const vec3& offset = vec3(0, 0, 0));

protected:
	virtual bool do_read(std::istream& in, AbstractMapBuilder& builder, bool use_provided_offset, const vec3& offset) ;
} ;



#endif

