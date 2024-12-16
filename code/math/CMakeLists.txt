get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


set(math_HEADERS
        box.h
        line.h
        math_types.h
        matrix.h
        plane.h
        polygon2d.h
        principal_axes.h
        semi_definite_symmetric_eigen.h
        vecg.h
        linear_program.h
        linear_program_solver.h
        )

set(math_SOURCES
        math_types.cpp
        polygon2d.cpp
        principal_axes.cpp
        semi_definite_symmetric_eigen.cpp
        linear_program_solver.cpp
        linear_program_solver_SCIP.cpp
        linear_program_solver_GUROBI.cpp
        )

add_library(${PROJECT_NAME} STATIC ${math_SOURCES} ${math_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "City3D")

target_include_directories(${PROJECT_NAME} PUBLIC
        ${POLYFIT_scip_DIR}
        ${POLYFIT_scip_DIR}/scip
        )

target_link_libraries( ${PROJECT_NAME} basic 3rd_scip 3rd_soplex)

include(../cmake/FindGUROBI.cmake)
if (GUROBI_FOUND)
    message(STATUS "GUROBI_INCLUDE_DIRS: ${GUROBI_INCLUDE_DIRS}")
    message(STATUS "GUROBI_LIBRARIES: ${GUROBI_LIBRARIES}")
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_GUROBI)
    target_include_directories(${PROJECT_NAME} PUBLIC ${GUROBI_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} basic ${GUROBI_LIBRARIES})
endif()

if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS _CRT_SECURE_NO_DEPRECATE)
endif()