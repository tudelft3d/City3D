cmake_minimum_required(VERSION 3.12)

project(city3d CXX)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
  ${CMAKE_BINARY_DIR}/lib CACHE
  PATH "Directory where all the .lib files are dumped." FORCE)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_BINARY_DIR}/bin CACHE
  PATH "Directory where .exe and .dll files are dumped." FORCE)

message("Building with CMake version: ${CMAKE_VERSION}")

add_executable(${PROJECT_NAME} src/main.cpp)
add_subdirectory(src/basic)
add_subdirectory(src/math)
add_subdirectory(src/method)
add_subdirectory(src/model)


# find_package(OpenMP REQUIRED)
# target_link_libraries(${PROJECT_NAME} PRIVATE llvm-openmp::llvm-openmp)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -pthread")

target_link_libraries(${PROJECT_NAME} PRIVATE basic math method model)
