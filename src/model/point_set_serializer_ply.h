
#ifndef _FILEIO_POINT_SERIALIZER_PLY_H_
#define _FILEIO_POINT_SERIALIZER_PLY_H_



#include <string>



class PointSet;

class PointSetSerializer_ply
{
public:
	static std::string title() { return "PointSetSerializer_ply"; }

	static void load(PointSet* pointSet, const std::string& file_name);
	static void	save(const PointSet* pointSet, const std::string& file_name, bool binary = true);
} ;

#endif

