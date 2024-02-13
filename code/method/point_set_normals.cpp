#include "point_set_normals.h"
#include "../model/point_set.h"
#include "../basic/logger.h"
#include "../model/kdtree_search.h"
#include "../basic/progress.h"
#include "../basic/stop_watch.h"


void PointSetNormals::estimate(PointSet* pset, int k /* = 16 */) {
	StopWatch t;
	t.start();

	KdTreeSearch_var kdtree = new KdTreeSearch;
	kdtree->begin();
	kdtree->add_vertex_set(pset);
	kdtree->end();

	int num = pset->num_points();
	const std::vector<vec3>& points = pset->points();
	std::vector<vec3>& normals = pset->normals();
	if (normals.size() != points.size())
		normals.resize(points.size());

	Logger::out("-") << "estimating normals ..." << std::endl;
	t.start();

	ProgressLogger progress(pset->num_points());
	for (int i = 0; i < num; ++i) {
		const vec3& p = points[i];
		std::vector<unsigned int> neighbors;
		kdtree->find_closest_K_points(p, k, neighbors); 

		PrincipalAxes3d pca;
		pca.begin() ;
		for (unsigned int j = 0; j < neighbors.size(); ++j) {
			int idx = neighbors[j];
			pca.add_point(points[idx]);
		}
		pca.end() ;

		// the eigen vector corresponding to the smallest eigen value
		normals[i] = pca.axis(2); 

		progress.next();
	}

	Logger::out("-") << "estimating normals done. Time: " << t.seconds() << " sec" << std::endl;
}

