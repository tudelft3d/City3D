cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set(
  ${PROJECT_NAME}_SOURCES
  main.cpp
)


add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "city3d_cli")

target_include_directories(${PROJECT_NAME} PRIVATE ${CITY3D_ROOT})

target_link_libraries(${PROJECT_NAME} basic math model method)

# The resources directory
target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE
  "CITY3D_ROOT_DIR=\"${CITY3D_ROOT}\""
)