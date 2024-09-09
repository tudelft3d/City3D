cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


set(
  math_HEADERS
  box.h
  line.h
  linear_program_solver.h
  linear_program.h
  math_types.h
  matrix.h
  plane.h
  polygon2d.h
  principal_axes.h
  semi_definite_symmetric_eigen.h
  vecg.h
)

set(
  math_SOURCES
  linear_program_solver_GUROBI.cpp
  linear_program_solver_SCIP.cpp
  linear_program_solver.cpp
  math_types.cpp
  polygon2d.cpp
  principal_axes.cpp
  semi_definite_symmetric_eigen.cpp
)

add_library(${PROJECT_NAME} STATIC ${math_SOURCES} ${math_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "City3D")

find_package(scip REQUIRED CONFIG)
if(scip_FOUND)
  message(STATUS "Found SCIP v${scip_VERSION}")
else()
  message(FATAL_ERROR "SCIP is required.")
endif()

find_package(soplex REQUIRED CONFIG)
if(soplex_FOUND)
  message(STATUS "Found soplex v${soplex_VERSION}")
else()
  message(FATAL_ERROR "soplex is required.")
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE basic scip::scip soplex)
