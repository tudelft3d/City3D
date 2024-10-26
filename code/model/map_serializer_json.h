
#ifndef _FILEIO_MESH_SERIALIZER_JSON_H_
#define _FILEIO_MESH_SERIALIZER_JSON_H_


#include "map_serializer.h"


// read Geojson format files using the single header file JSON library:
// https://github.com/nlohmann/json
// I store 2D polygons as a 3D mesh (all z-coordinates are set to 0) consisting of
// a set of individual faces.


class MapSerializer_json : public MapSerializer
{
public:
	MapSerializer_json();

    virtual bool binary() const override;

protected:
	virtual bool do_read(std::istream& in, AbstractMapBuilder& builder) ;
} ;



#endif

