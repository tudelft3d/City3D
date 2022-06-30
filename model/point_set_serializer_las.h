
#ifndef _FILEIO_POINT_SERIALIZER_LAS_H_
#define _FILEIO_POINT_SERIALIZER_LAS_H_

#include "model_common.h"

#include <string>



class PointSet;

// for both las and laz formats
// internally it uses the LASlib of martin.isenburg@rapidlasso.com. 
// For more information, see http://rapidlasso.com

class MODEL_API PointSetSerializer_las
{
public:
	static void load(PointSet* pointSet, const std::string& file_name);
} ;

#endif

