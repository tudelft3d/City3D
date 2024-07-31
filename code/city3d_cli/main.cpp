
#include "../basic/file_utils.h"
#include "../basic/input_parser.h"
#include "../basic/logger.h"
#include "../method/reconstruction.h"
#include "../model/map.h"
#include "../model/map_io.h"
#include "../model/point_set.h"
#include "../model/point_set_io.h"
#include <getopt.h>

int main(int argc, char **argv) {
  Logger::initialize();
  Logger::instance()->set_value(Logger::LOG_REGISTER_FEATURES,
                                "*"); // log everything
  // parse and validate input parameters
  InputParser input(argc, argv);
  if (input.cmd_option_exists("-h")) {
    std::cout << "Help using city3d_cli" << std::endl;
    std::cout << "-f Path to the building footprint (.obj)" << std::endl;
    std::cout << "-p Path to the building pointcloud data (.ply)" << std::endl;
    std::cout << "-o Path to the output result (.obj)" << std::endl;
    return 0;
  }
  if (!input.cmd_option_exists("-f")) {
    std::cerr << "Path to the footprint is missing (-f)" << std::endl;
    return EXIT_FAILURE;
  }
  if (!input.cmd_option_exists("-p")) {
    std::cerr << "Path to the pointcloud is missing (-p)" << std::endl;
    return EXIT_FAILURE;
  }
  if (!input.cmd_option_exists("-o")) {
    std::cerr << "Path to the output result is missing (-o)" << std::endl;
    return EXIT_FAILURE;
  }
  const std::string &footprint_file = input.get_cmd_option("-f");
  const std::string &pointcloud_file = input.get_cmd_option("-p");
  const std::string &output_file = input.get_cmd_option("-o");
  if (footprint_file.empty()) {
    std::cerr << "Empty path given (-f)" << std::endl;
    return EXIT_FAILURE;
  }
  if (pointcloud_file.empty()) {
    std::cerr << "Empty path given (-p)" << std::endl;
    return EXIT_FAILURE;
  }
  if (!FileUtils::is_file(footprint_file)) {
    std::cerr << "Error in file path: " << footprint_file << " is not a file.";
    return EXIT_FAILURE;
  }
  if (!FileUtils::is_file(pointcloud_file)) {
    std::cerr << "Error in file path: " << pointcloud_file << " is not a file.";
    return EXIT_FAILURE;
  }

  // load input point cloud
  std::cout << "loading input point cloud data... (from file: "
            << pointcloud_file << ")" << std::endl;
  PointSet *pset = PointSetIO::read(pointcloud_file);

  if (!pset) {
    std::cerr << "failed loading point cloud data from file " << pointcloud_file
              << std::endl;
    return EXIT_FAILURE;
  }

  // load input footprint data
  std::cout << "loading input footprint data..." << std::endl;
  Map *footprint = MapIO::read(footprint_file);
  if (!footprint) {
    std::cerr << "failed loading footprint data from file " << footprint_file
              << std::endl;
    return EXIT_FAILURE;
  }

  Reconstruction recon;

  // Step 1: segmentation to obtain point clouds of individual buildings
  std::cout << "segmenting individual buildings..." << std::endl;
  recon.segmentation(pset, footprint);

  // Step 2: extract planes from point cloud
  std::cout << "extracting roof planes..." << std::endl;
  int num_roofs = recon.extract_roofs(pset, footprint);
  if (num_roofs <= 0) {
    std::cerr << "failed extracting roof planes from file " << pointcloud_file
              << std::endl;
    return EXIT_FAILURE;
  }
  std::cerr << "extracted " << num_roofs << " roof "
            << (num_roofs > 1 ? "planes" : "plane") << std::endl;

  // Step 3: reconstruction of all the buildings in the scene
  Map *result = new Map;

#ifdef HAS_GUROBI
  std::cout << "reconstructing the buildings (using the Gurobi solver)..."
            << std::endl;
  bool status =
      recon.reconstruct(pset, footprint, result, LinearProgramSolver::GUROBI);
#else
  std::cout << "reconstructing the buildings (using the SCIP solver)..."
            << std::endl;
  bool status =
      recon.reconstruct(pset, footprint, result, LinearProgramSolver::SCIP);
#endif

  if (status && result->size_of_facets() > 0) {
    if (MapIO::save(output_file, result)) {
      std::cout << "reconstruction result saved to file " << output_file
                << std::endl;
      return EXIT_SUCCESS;
    } else
      std::cerr << "failed to save reconstruction result to file "
                << output_file << std::endl;
  }
  std::cerr << "reconstruction failed" << std::endl;

  delete pset;
  delete footprint;
  delete result;

  return EXIT_FAILURE;
}
