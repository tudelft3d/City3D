
#include "map_serializer_json.h"
#include "map_builder.h"
#include "map_enumerator.h"
#include "json.hpp"
#include "../basic/logger.h"

// see docs: https://nlohmann.github.io/json/

// for convenience
using json = nlohmann::json;

MapSerializer_json::MapSerializer_json()
{
    read_supported_ = true ;
    write_supported_ = false ;
}

bool MapSerializer_json::binary() const {
    return false;
}


void extract_polygon(json::const_iterator it_coordinates, std::vector<double>& coordinates) {
	for (auto it = it_coordinates->begin(); it != it_coordinates->end(); ++it) {
		if (it->is_array())
			extract_polygon(it, coordinates);
		else if (it->is_number_float())
			coordinates.push_back(*it);
	}

	// remove very close points
	std::vector<double> results;
	double cur_x = 0;
	double cur_y = 0;
	for (int i=0; i<coordinates.size(); i+=2) {
		if (i+1 >= coordinates.size())
			continue;
		if (i==0) {
			cur_x = coordinates[i];
			cur_y = coordinates[i + 1];
			results.push_back(cur_x);
			results.push_back(cur_y);
		}
		else {
			double x = coordinates[i];
			double y = coordinates[i + 1];
			if (std::abs(x -cur_x) > 1e-5 || std::abs(y - cur_y) > 1e-5) {
				cur_x = x;
				cur_y = y;
				results.push_back(cur_x);
				results.push_back(cur_y);
			}
			else {
				std::cout << "duplicated points: (" << x << ", " << y << ")" << std::endl;
			}
		}
	}
	coordinates = results;
}


bool MapSerializer_json::do_read(std::istream& in, AbstractMapBuilder& builder)
{
	Logger::warn("-") << "The GeoJSON parser is not fully implemented and it may not handle all files!" << std::endl;

	// read a JSON file
	json object;
	in >> object;

	builder.begin_surface();
	int idx = 0;

	auto it_features = object.find("features");
	if (it_features == object.end())
		return false;
	if (!it_features->is_array())
		return false;

	for (std::size_t i = 0; i < it_features->size(); ++i) {
		const json& ei = it_features->at(i);
		if (!ei.is_object() || ei["type"] != "Feature")
			continue;

		auto it_geometry = ei.find("geometry"); // you can also use if (iter.key() == "geometry")
		if (it_geometry == ei.end() || !it_geometry->is_object())
			continue;

		auto it_coordinates = it_geometry->find("coordinates");
		if (it_coordinates == it_geometry->end() || !it_coordinates->is_array())
			continue;

		auto it_type = it_geometry->find("type");
		if (it_type == it_geometry->end() || !it_type.value().is_string())
			continue;

		if (it_type.value() == "MultiPolygon" || it_type.value() == "Polygon") {
			std::vector<double> coordinates;
			extract_polygon(it_coordinates, coordinates);

			builder.begin_facet();
            // The GeoJSON specification says:
            //      The first and last positions are equivalent, and they MUST contain
            //      identical values; their representation SHOULD also be identical.
            // Thus the "-2", see https://www.rfc-editor.org/rfc/rfc7946.html#section-3.1.6
			for (std::size_t j = 0; j < coordinates.size()-2; j += 2) {
				builder.add_vertex(vec3(coordinates[j], coordinates[j + 1], 0));
				builder.add_vertex_to_facet(idx);
				++idx;
			}
			builder.end_facet();
		}
	}

	builder.end_surface();

	return true;
}
