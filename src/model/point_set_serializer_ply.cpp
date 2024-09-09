
#include "point_set_serializer_ply.h"
#include "rply.h"
#include "point_set.h"
#include "../basic/logger.h"
#include "../basic/basic_types.h"
#include "../basic/logger.h"

#include <cassert>


inline static bool is_big_endian() {
	int i = 1;
	char *p = reinterpret_cast<char *>(&i);
	if (p[0] == 1) // Lowest address contains the least significant byte
		return false; // LITTLE_ENDIAN
	else
		return true; // BIG_ENDIAN
}

class PlyPointSetLoad 
{
public:
	void load(PointSet* pointSet, const std::string& filename) {
		p_ply ply = ply_open(filename.c_str(), nil, 0, nil) ;
		if(ply == nil) {
			Logger::err("PlyPointSetLoad") << filename << ": could not open" << std::endl;
			return ;
		}

		if(!ply_read_header(ply)) {
			Logger::err("PlyPointSetLoad") << filename << ": invalid PLY file" << std::endl;
			ply_close(ply) ;
			return ;
		}

		point_set_ = pointSet;

		current_vertex_ = 0 ;
		current_color_  = 0 ;
		current_normal_ = 0 ;
		check_for_colors_and_normals(ply) ;

		long nvertices = ply_set_read_cb(ply, "vertex", "x", PlyPointSetLoad::vertex_cb, this, 0) ;
		ply_set_read_cb(ply, "vertex", "y", PlyPointSetLoad::vertex_cb, this, 1) ;
		ply_set_read_cb(ply, "vertex", "z", PlyPointSetLoad::vertex_cb, this, 2) ;

		create_vertices(nvertices) ;

		if(!ply_read(ply)) {
			Logger::err("PlyPointSetLoad") 
				<< filename << ": problem occurred while parsing PLY file" << std::endl;
		}

		ply_close(ply) ;
		end_point_set() ;
	}

protected:
	void check_for_colors_and_normals(p_ply ply) {
		p_ply_element element = nil ;

		std::string str_red, str_green, str_blue;
		
		bool has_normals = false;
		bool has_normals_Neil = false;

		for(;;) {
			element = ply_get_next_element(ply, element) ;
			if(element == nil) { break ; }
			const char* elt_name = nil ;
			ply_get_element_info(element, &elt_name, nil) ;

			if(!strcmp(elt_name, "vertex")) {
				p_ply_property property = nil ;
				for(;;) {
					property = ply_get_next_property(element, property) ;
					if(property == nil) 
						break ;

					const char* prop_name = nil ;
					ply_get_property_info(property, &prop_name, nil, nil, nil) ;
					if (!strcmp(prop_name, "r"))	str_red = "r" ;
					if (!strcmp(prop_name, "g"))	str_green = "g" ;
					if (!strcmp(prop_name, "b"))	str_blue = "b" ;
					if (!strcmp(prop_name, "red"))		str_red = "red" ;
					if (!strcmp(prop_name, "green"))	str_green = "green" ;
					if (!strcmp(prop_name, "blue"))		str_blue = "blue" ;
					if (!strcmp(prop_name, "diffuse_red"))		str_red = "diffuse_red" ;
					if (!strcmp(prop_name, "diffuse_green"))	str_green = "diffuse_green" ;
					if (!strcmp(prop_name, "diffuse_blue"))		str_blue = "diffuse_blue" ;

					has_normals  = has_normals || !strcmp(prop_name, "nx");
					has_normals  = has_normals || !strcmp(prop_name, "ny");
					has_normals  = has_normals || !strcmp(prop_name, "nz");
					has_normals_Neil  = has_normals || !strcmp(prop_name, "vsfm_cnx");  // for Neil Smith's
					has_normals_Neil  = has_normals || !strcmp(prop_name, "vsfm_cny");
					has_normals_Neil  = has_normals || !strcmp(prop_name, "vsfm_cnz");
				}
			} 
		}

		if(str_red == "r" && str_green == "g" && str_blue == "b") {
			has_colors_ = true ;
			color_mult_ = 1.0f ;
		} 
		else if (
			(str_red == "red" && str_green == "green" && str_blue == "blue") ||
			(str_red == "diffuse_red" && str_green == "diffuse_green" && str_blue == "diffuse_blue") 
			)
		{
			has_colors_ = true ;
			color_mult_ = 1.0f / 255.0f ;
		} 
		else {
			has_colors_ = false ;
		}

		if (has_colors_) {
			ply_set_read_cb(ply, "vertex", str_red.c_str(),   PlyPointSetLoad::color_cb, this, 0) ;
			ply_set_read_cb(ply, "vertex", str_green.c_str(), PlyPointSetLoad::color_cb, this, 1) ;
			ply_set_read_cb(ply, "vertex", str_blue.c_str(),  PlyPointSetLoad::color_cb, this, 2) ;
		}

		has_normals_ = has_normals;
		if (has_normals) {
			has_normals_ = true;
			ply_set_read_cb(ply, "vertex", "nx",   PlyPointSetLoad::normal_cb, this, 0) ;
			ply_set_read_cb(ply, "vertex", "ny",   PlyPointSetLoad::normal_cb, this, 1) ;
			ply_set_read_cb(ply, "vertex", "nz",   PlyPointSetLoad::normal_cb, this, 2) ;
		} 
		else if (has_normals_Neil) {
			has_normals_ = true;
			ply_set_read_cb(ply, "vertex", "vsfm_cnx",   PlyPointSetLoad::normal_cb, this, 0) ;
			ply_set_read_cb(ply, "vertex", "vsfm_cny",   PlyPointSetLoad::normal_cb, this, 1) ;
			ply_set_read_cb(ply, "vertex", "vsfm_cnz",   PlyPointSetLoad::normal_cb, this, 2) ;
		} 
		else
			has_normals_ = false;
	}

	static PlyPointSetLoad* plyload(p_ply_argument argument) {
		PlyPointSetLoad* result = nil ;
		ply_get_argument_user_data(argument, (void**)(&result), nil) ;
		assert(result != nil) ;
		return result ;
	}

	static int vertex_cb(p_ply_argument argument) {
		return plyload(argument)->add_vertex_data(argument) ;
	}

	static int color_cb(p_ply_argument argument) {
		return plyload(argument)->add_color_data(argument) ;
	}

	static int normal_cb(p_ply_argument argument) {
		return plyload(argument)->add_normal_data(argument) ;
	}

	int add_vertex_data(p_ply_argument argument) {
		long coord ;
		ply_get_argument_user_data(argument, nil, &coord);
		assert(coord >= 0 && coord < 3) ;
		xyz_[coord] = (ply_get_argument_value(argument)) ;
		if(coord == 2) { 
			(*vertices_)[current_vertex_] = vec3(xyz_[0], xyz_[1], xyz_[2]);
			current_vertex_++ ; 
		}
		return 1;
	}

	int add_color_data(p_ply_argument argument) {
		long coord ;
		ply_get_argument_user_data(argument, nil, &coord);
		assert(coord >= 0 && coord < 3) ;
		rgb_[coord] = (ply_get_argument_value(argument)) * color_mult_ ;
		if(coord == 2) { 
			set_vertex_color(current_color_, vec3(rgb_)) ;
			current_color_++ ; 
		}
		return 1 ;
	}

	int add_normal_data(p_ply_argument argument) {
		long coord ;
		ply_get_argument_user_data(argument, nil, &coord);
		assert(coord >= 0 && coord < 3) ;
		normal_[coord] = (ply_get_argument_value(argument));
		if(coord == 2) { 
			set_vertex_normal(current_normal_, vec3((normal_[0]), (normal_[1]), (normal_[2]))) ;
			current_normal_++ ; 
		}
		return 1 ;
	}

	//////////////////////////////////////////////////////////////////////////
	
	void create_vertices(unsigned int nb_vertices) {
		vertices_ = &(point_set_->points());
		vertices_->resize(nb_vertices);

		if (has_normals_) {
			normals_ = &(point_set_->normals());
			normals_->resize(nb_vertices);
		}

		if (has_colors_){
			colors_ = &(point_set_->colors());
			colors_->resize(nb_vertices);
		}
	}

	void set_vertex_color(unsigned int idx, const vec3& c) {
		if(idx < 0 || idx >= int(vertices_->size())) {
			Logger::warn("PlyPointSetLoad") << "vertex index " << idx << " out of range" << std::endl;
			return ;
		}

		colors_->at(idx) = c;
	}

	void set_vertex_normal(unsigned int idx, const vec3& n) {
		if (idx < 0 || idx >= int(vertices_->size())) {
			Logger::warn("PlyPointSetLoad") << "vertex index " << idx << " out of range" << std::endl;
			return ;
		}

		normals_->at(idx) = n;
	}

	void end_point_set() {
		point_set_ = nil;

		current_vertex_ = 0;
		current_color_ = 0;
	}

protected:
	PointSet*			point_set_;
	std::vector<vec3>*  vertices_;
	std::vector<vec3>*	normals_ ;
	std::vector<vec3>*	colors_;

	double			xyz_[3];
	unsigned int	current_vertex_ ;

	bool			has_colors_ ;
	double			color_mult_ ;
	float			rgb_[3];
	unsigned int	current_color_ ;

	bool			has_normals_;
	unsigned int	current_normal_ ;
	double			normal_[3];
} ;

//__________________________________________________________

void PointSetSerializer_ply::load(PointSet* pointSet, const std::string& file_name) {
	PlyPointSetLoad loader ;
	loader.load(pointSet, file_name) ;
}


class PlyPointSetSave {
public:
	PlyPointSetSave() : color_mult_(255.0) { }

	bool save(const PointSet* model, const std::string& filename, bool binary) {
		e_ply_storage_mode storage = PLY_ASCII;
		if (binary) 
			storage = is_big_endian() ? PLY_BIG_ENDIAN : PLY_LITTLE_ENDIAN;

		p_ply ply = ply_create(filename.c_str(), storage, nil, 0, nil);

		if(ply == nil) {
			Logger::err("PlyPointSetSave") << filename << ": could not open" << std::endl;
			return false ;
		}

		//////////////////////////////////////////////////////////////////////////

		if (!ply_add_comment(ply, "saved by liangliang.nan@gmail.com")) {
			Logger::err("PlyPointSetSave") << "unable to add comment" << std::endl;
			ply_close(ply) ;
			return false ;
		}

		object_ = model;
		int num_v = int(object_->num_points());
		if (!ply_add_element(ply, "vertex", num_v)) {
			Logger::err("PlyPointSetSave") << "unable to add element \'vertex\'" << std::endl;
			ply_close(ply) ;
			return false ;
		}

		e_ply_type length_type, value_type;
		length_type = value_type = static_cast<e_ply_type>(-1);
		std::string pos[3] = { "x", "y", "z" };
		for (unsigned int i=0; i<3; ++i) {
			if (!ply_add_property(ply, pos[i].c_str(), PLY_FLOAT, length_type, value_type)) {
				Logger::err("PlyPointSetSave") << "unable to add property \'" << pos[i] << "\'" << std::endl;
				ply_close(ply) ;
				return false ;
			}
		}
	
		const std::vector<vec3>& normals = object_->normals();
		const std::vector<vec3>& colors = object_->colors();
		bool has_normals = object_->has_normals();
		bool has_colors = object_->has_colors();
		if (has_normals) {
			std::string normal[3] = { "nx", "ny", "nz" };
			for (unsigned int i=0; i<3; ++i) {
				if (!ply_add_property(ply, normal[i].c_str(), PLY_FLOAT, length_type, value_type)) {
					Logger::err("PlyPointSetSave") << "unable to add property \'" << pos[i] << "\'" << std::endl;
					ply_close(ply) ;
					return false ;
				}
			}
		}
		if (has_colors) {
			std::string color[3] = { "red", "green", "blue"};
			for (unsigned int i=0; i<3; ++i) {
				if (!ply_add_property(ply, color[i].c_str(), PLY_UCHAR, length_type, value_type)) {
					Logger::err("PlyPointSetSave") << "unable to add property \'" << color[i] << "\'" << std::endl;
					ply_close(ply) ;
					return false ;
				}
			}
		}
		
		if(!ply_write_header(ply)) {
			Logger::err("PlyPointSetSave") << filename << ": invalid PLY file" << std::endl;
			ply_close(ply) ;
			return false ;
		}

		//////////////////////////////////////////////////////////////////////////

// 		ProgressLogger progress(object_->size_of_vertices());
		const std::vector<vec3>& points = object_->points();
		for (std::size_t idx = 0; idx < points.size(); ++idx) {
// 			progress.notify(idx);

			const vec3& p = points[idx];
			ply_write(ply, p.x);
			ply_write(ply, p.y);
			ply_write(ply, p.z);

			if (has_normals) {
				const vec3& n = normals[idx];
				ply_write(ply, n.x);
				ply_write(ply, n.y);
				ply_write(ply, n.z);
			} 
			
			if (has_colors) {
				const vec3& c = colors[idx];
				float r = c.x * color_mult_;	ogf_clamp(r, 0.0f, 255.0f);
				float g = c.y * color_mult_;	ogf_clamp(g, 0.0f, 255.0f);
				float b = c.z * color_mult_;	ogf_clamp(b, 0.0f, 255.0f);
				ply_write(ply, r);
				ply_write(ply, g);
				ply_write(ply, b);
			}
		}
		
		ply_close(ply);
		return true ;
	}

protected:
	const PointSet*	object_;
	float			color_mult_;
} ;


void PointSetSerializer_ply::save(const PointSet* model, const std::string& file_name, bool binary/* = true*/) {
	PlyPointSetSave plysave ;
	plysave.save(model, file_name, binary);
}