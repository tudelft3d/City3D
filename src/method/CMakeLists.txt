cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set(
  method_HEADERS
  alpha_shape.h
  alpha_shape_boundary.h
  alpha_shape_mesh.h
  cgal_types.h
  dbscan.h
  face_selection.h
  hypothesis_generator.h
  intersection_area.h
  method_global.h
  Modified_Otr2.h
  otr2_edge_simplify.h
  point_set_normals.h
  point_set_region_growing.h
  polyfit_info.h
  reconstruction.h
  regularize_polygon.h
)

set(
  method_SOURCES
  alpha_shape_mesh.cpp
  alpha_shape_boundary.cpp
  face_selection.cpp
  face_selection_optimization.cpp
  point_set_normals.cpp
  hypothesis_generator.cpp
  polyfit_info.cpp
  reconstruction.cpp
  method_global.cpp
  point_set_region_growing.cpp
  regularize_polygon.cpp
  otr2_edge_simplify.cpp
)

add_library(${PROJECT_NAME} STATIC ${method_SOURCES} ${method_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "City3D")

find_package(cgal REQUIRED)
if(cgal_FOUND)
  message(STATUS "Found CGAL v${cgal_VERSION}")
else()
  message(FATAL_ERROR "CGAL is required.")
endif()

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  message(STATUS "Found Eigen3 v${Eigen3_VERSION}")
else()
  message(FATAL_ERROR "Eigen3 is required.")
endif()
include(CGAL_Eigen3_support)

find_package(gmp REQUIRED CONFIG)
if(gmp_FOUND)
  message(STATUS "Found gmp v${gmp_VERSION}")
else()
  message(FATAL_ERROR "gmp is required.")
endif()

find_package(scip REQUIRED CONFIG)
if(scip_FOUND)
  message(STATUS "Found SCIP v${scip_VERSION}")
else()
  message(FATAL_ERROR "SCIP is required.")
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::CGAL CGAL::Eigen3_support gmp::libgmp scip::scip)

find_package(OpenCV COMPONENTS REQUIRED core imgproc CONFIG)
if(OpenCV_FOUND)
  message(STATUS "Found OpenCV v${OpenCV_VERSION}")
  message(STATUS "Linking ${OpenCV_LIBS}")
else()
  message(FATAL_ERROR "OpenCV is required.")
endif()

find_package(OpenMP REQUIRED)

target_link_libraries(${PROJECT_NAME} PRIVATE basic math model ${OpenCV_LIBS} llvm-openmp::llvm-openmp)