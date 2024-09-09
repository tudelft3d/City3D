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

#include "stop_watch.h"
#include "basic_types.h" // for "round()"

#include <cmath>
#include <sstream>
#include <iomanip>


//_________________________________________________________

StopWatch::StopWatch() {
	start();
}

StopWatch::~StopWatch() {}


void StopWatch::start() {
#ifdef WIN32
	LARGE_INTEGER  largeInteger;
	QueryPerformanceFrequency(&largeInteger);
	freq_ = largeInteger.QuadPart;
	QueryPerformanceCounter(&largeInteger);
	start_count_ = largeInteger.QuadPart;
#else
	gettimeofday(&start_time_, 0);
#endif // WIN32
}

double StopWatch::seconds() const {
#ifdef WIN32
	LARGE_INTEGER  largeInteger;
	QueryPerformanceCounter(&largeInteger);
	LONGLONG now_count = largeInteger.QuadPart;
	double time = (double)( (now_count - start_count_) / static_cast<double>(freq_) );
#else
	timeval now;
	gettimeofday(&now, 0);
    double time = (now.tv_sec - start_time_.tv_sec) + (now.tv_usec - start_time_.tv_usec) / 1.0e6;
#endif  // WIN32
    return truncate_digits(time, 2);
}


std::string StopWatch::time_string(int num_digits /* = 1*/) const {
	double time = seconds() * 1000;
	if (std::isnan(time) || std::isinf(time))
		return "inf";

	std::string suffix = "ms";
	if (time > 1000) {
		time /= 1000; suffix = "s";
		if (time > 60) {
			time /= 60; suffix = "m";
			if (time > 60) {
				time /= 60; suffix = "h";
				if (time > 12) {
					time /= 12; suffix = "d";
				}
			}
		}
	}

	std::ostringstream os;
	os << std::setprecision(num_digits)
		<< std::fixed << time << suffix;

	return os.str();
}