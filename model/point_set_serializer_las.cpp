
#include "point_set_serializer_las.h"
#include "../basic/logger.h"
#include "../basic/basic_types.h"
#include "../basic/logger.h"
#include "point_set.h"
#include "../3rd_party/3rd_LAStools/LASlib/inc/lasreader.hpp"
#include "../3rd_party/3rd_LAStools/LASlib/inc/laswriter.hpp"

#include <cassert>
#include <algorithm>
#include <climits>


void PointSetSerializer_las::load(PointSet* pset, const std::string& file_name) {
	LASreadOpener lasreadopener;
	lasreadopener.set_file_name(file_name.c_str(), true);

	LASreader* lasreader = lasreadopener.open();
	if (!lasreader || lasreader->npoints <= 0) {
		Logger::err("-") << "could not open file" << std::endl;
		lasreader->close();
		delete lasreader;
		return;
	}

	std::size_t num = lasreader->npoints;
	std::vector<vec3>& points = pset->points();
	std::vector<vec3>& colors = pset->colors();
	points.resize(num);
	colors.resize(num);
	if (points.size() != num || colors.size() != num) {
		Logger::err("-") << "failed allocating memory for " << num << " points" << std::endl;
		points.clear();
		colors.clear();
		lasreader->close();
		delete lasreader;
		return;
	}

	Logger::out("-") << "reading " << num  << " points..." << std::endl;

	// read the first point
	if (!lasreader->read_point()) {
		Logger::err("-") << "failed reading point" << std::endl;
		points.clear();
		colors.clear();
		lasreader->close();
		delete lasreader;
		return;
	}

	// Liangliang: las format usually represent very large area of urban scenes and some coordinates
	//			   may have very large values. In order to render the points properly in OpenGL, I
	//			   record the relative positions to the first point stored in the file.
	std::size_t idx = 0;
	LASpoint& p0 = lasreader->point;
	points[idx] = vec3(0, 0, 0);
	float r, g, b;
	if (p0.have_rgb) { // some file may have rgb
		r = float(p0.get_R()) / USHRT_MAX;
		g = float(p0.get_G()) / USHRT_MAX;
		b = float(p0.get_B()) / USHRT_MAX;
	}
	else // in case color doesn't exist, use intensity
		r = g = b = p0.intensity % 255 / 255.0f;
	colors[idx] = vec3(r, g, b);
	++idx;

	// compute the actual coordinates as double floating point values
	p0.compute_coordinates();
	double x0 = p0.coordinates[0];
	double y0 = p0.coordinates[1];
	double z0 = p0.coordinates[2];
	Logger::out("-") << "first point (" << x0 << " " << y0 << " " << z0 << ")" << std::endl;
	pset->set_offset(vec3(x0, y0, z0));

	// now we read the remaining points...
	while (lasreader->read_point()) {
		LASpoint& p = lasreader->point;

		// compute the actual coordinates as double floating point values
		p.compute_coordinates();
		double x = p.coordinates[0] - x0;
		double y = p.coordinates[1] - y0;
		double z = p.coordinates[2] - z0;
		points[idx] = vec3(x, y, z);

		float r, g, b;
		if (p.have_rgb) { // some file may have rgb
			r = float(p.get_R()) / USHRT_MAX;
			g = float(p.get_G()) / USHRT_MAX;
			b = float(p.get_B()) / USHRT_MAX;
		}
		else // in case color doesn't exist, use intensity
			r = g = b = p.intensity % 255 / 255.0f;

		colors[idx] = vec3(r, g, b);
		++idx;
	}

	lasreader->close();
	delete lasreader;
}