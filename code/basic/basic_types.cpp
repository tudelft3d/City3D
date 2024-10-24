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


#include "basic_types.h"

#include <cmath>
#include <cctype>
#include <ctime>
#include <chrono>
#include <iomanip>

#ifdef WIN32
#include <cfloat> // for ::_isnan()
#else
#include <cstdlib>
#endif




namespace Numeric {
	const float big_float = 1e10f ;
	const float small_float = 1e-10f ;
	const double big_double = 1e20 ;
	const double small_double = 1e-20 ;

	// Note: do not add global scole (::isnan) for calling this function, 
	// since it is a macro in some implementations. 

	bool is_nan(float32 x) {
#ifdef WIN32
		return (_isnan(x) != 0) || (_finite(x) == 0) ;
#else
        return std::isnan(x) || !std::isfinite(x);
#endif
	}

	bool is_nan(float64 x) {
#ifdef WIN32
		return (_isnan(x) != 0) || (_finite(x) == 0) ;
#else
        return std::isnan(x) || !std::isfinite(x);
#endif
	}    

	int32 random_int32() {
#ifdef WIN32
		return rand() ;
#else
		return random() ;
#endif        
	}

	float32 random_float32() {
#ifdef WIN32
		return float(rand()) / float(RAND_MAX);
#else
		return float(drand48()) ;
#endif        
	}

	float64 random_float64() {
#ifdef WIN32
		return double(rand()) / double(RAND_MAX);
#else
		return double(drand48()) ;
#endif        
	}


}

namespace String {

	void split_string(
		const std::string& in, 
		char separator,
		std::vector<std::string>& out,
		bool skip_empty_fields/* = true */) 
	{
		size_t length = in.length() ;
		size_t start = 0 ;
		size_t end = 0 ;

		while(start < length) {

			end = in.find(separator,start) ;
			if(end == std::string::npos) {
				end = length ;
			}

			if(!skip_empty_fields || (end - start > 0)) {
				out.push_back(in.substr(start, end - start)) ;
			}
			start = end + 1 ;
		}
	}


	void join_strings(
		const std::vector<std::string>& in,
		char separator,
		std::string& out
		) {
			out = "" ;
			for(unsigned int i=0; i<in.size(); i++) {
				if(out.length() != 0) {
					out += separator ;
				}
				out += in[i] ;
			}
	}

	void join_strings(
		const std::vector<std::string>& in,
		const std::string& separator,
		std::string& out
		) {
			out = "" ;
			for(unsigned int i=0; i<in.size(); i++) {
				if(out.length() != 0) {
					out += separator ;
				}
				out += in[i] ;
			}
	}

	std::string join_strings(
		const std::vector<std::string>& in,
		char separator
		) {
			std::string result ;
			join_strings(in, separator, result) ;
			return result ;
	}

	std::string join_strings(
		const std::vector<std::string>& in,
		const std::string& separator
		) {
			std::string result ;
			join_strings(in, separator, result) ;
			return result ;
	}

	int replace_substring(
		std::string& str, 
		const std::string& original_phrase, 
		const std::string& new_phrase,
		bool isolated /* = true*/
		) {
			// Prevent infinite loop : if original_phrase is empty, do nothing and return false
			if (original_phrase.empty()) 
				return false;

			int replaced_count = 0;
			std::string::size_type pos = 0;
			while((pos=str.find(original_phrase, pos))!=std::string::npos)
			{
				if (isolated) { // check if the substr is isolated by spaces
					char space = ' ';
					std::string::size_type endOfPhrasePos = pos+original_phrase.size();

					if (pos < endOfPhrasePos && endOfPhrasePos < str.size()) {
						if (pos > 0) {
							char pre = str[pos - 1];
							if (pre != space) {
								pos = endOfPhrasePos;
								continue;
							}
						}

						char suf = str[endOfPhrasePos];
						if (suf != space) {
							pos = endOfPhrasePos;
							continue;
						}
					}
				}

				str.replace(pos, original_phrase.size(), new_phrase);
				++replaced_count;
			}
			return replaced_count;
	}
	
	void to_lowercase(std::string& in) {
		for(unsigned int i=0; i<in.length(); i++) {
			if(isupper(in[i])) {
				in[i] = tolower(in[i]) ;
			}
		}
	}

	void to_uppercase(std::string& in) {
		for(unsigned int i=0; i<in.length(); i++) {
			if(islower(in[i])) {
				in[i] = toupper(in[i]) ;
			}
		}
	}        

	std::string quote(const std::string& s, char quotes) {
		return char_to_string(quotes) + s + char_to_string(quotes) ;
	}

    // format: "Fri Jan 09 11:39:32 2015"
    std::string current_time() {
        // Get the current time as a time_t object
        std::time_t now = std::time(nullptr);
        // Convert the time_t to a tm structure
        std::tm* localTime = std::localtime(&now);
        // Use a stringstream to format the time
        std::stringstream ss;
        ss << std::put_time(localTime, "%a %b %d %H:%M:%S %Y");
        return ss.str();
    }

    // format: "2024-10-24-17-41-16-753"
    std::string current_time_detailed() {
        const auto now = std::chrono::system_clock::now();
        const auto now_as_time_t = std::chrono::system_clock::to_time_t(now);
        const auto now_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        std::stringstream time_stream;
        time_stream << std::put_time(std::localtime(&now_as_time_t), "%Y-%m-%d-%H-%M-%S") // format like "2024-10-24-17-41-16-753"
                    << '-' << std::setfill('0') << std::setw(3)
                    << now_in_ms.count(); // add milliseconds
        return time_stream.str();
    }

}

