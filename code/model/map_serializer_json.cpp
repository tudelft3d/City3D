
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
}

Map* MapSerializer_json::read(const std::string& file_name, bool use_provided_offset /* = false */, const vec3& offset /* = vec3(0, 0, 0) */ )
{
	std::fstream::openmode mode = std::fstream::in;

	std::ifstream input(file_name.c_str(), mode);
	if (input.fail()) {
		Logger::err("-")
			<< "Could not open file\'"
			<< file_name << "\'"
			<< std::endl;
		return nullptr;
	}

	Map* mesh = new Map;
	MapBuilder builder(mesh);
	if (do_read(input, builder, use_provided_offset, offset))
		return mesh;
	else {
		delete mesh;
		return nullptr;
	}
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


bool MapSerializer_json::do_read(std::istream& in, AbstractMapBuilder& builder, bool use_provided_offset /* = false */, const vec3& offset /* = vec3(0, 0, 0) */)
{
	Logger::warn("-") << "I can handle only simple GeoJSON format!" << std::endl;

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

	bool first_point = true;
	double dx = offset.x;
	double dy = offset.y;
	double dz = offset.z;
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
			for (std::size_t j = 0; j < coordinates.size()-2; j += 2) {
				if (!use_provided_offset && first_point) {
					dx = coordinates[j];
					dy = coordinates[j + 1];
					first_point = false;
				}

				builder.add_vertex(vec3(coordinates[j] - dx, coordinates[j + 1] - dy, 0 - dz));
				
				builder.add_vertex_to_facet(idx);
				++idx;
			}
			builder.end_facet();
		}
	}

	builder.end_surface();

	return true;
}
