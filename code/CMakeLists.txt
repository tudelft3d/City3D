cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(City3D)

if (POLICY CMP0074)
        cmake_policy(SET CMP0074 NEW)
endif ()

# One shouldn't generate the BUILD project directly in the SOURCES folder!
if ( ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
        if ( NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED )
                message(FATAL_ERROR "It is not advised to BUILD the binaries directly in the SOURCE folder!\n If you want to proceed with this option, just CONFIGURE the project once again" )
                set( SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE )
        endif()
endif()

################################################################################

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message("-- Setting C++11")

if(MSVC)
        # The /MP flag is available and works as intended with the MSVC compiler
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
elseif(CMAKE_BUILD_PARALLEL_LEVEL)
        # Use build system parallelism (e.g., `make -j` or `ninja`)
        set(CMAKE_BUILD_PARALLEL_LEVEL 8) # Example: Use 8 threads
endif()

################################################################################

# Detects whether this is a top-level project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        set(POLYFIT_TOPLEVEL_PROJECT ON)
else()
        set(POLYFIT_TOPLEVEL_PROJECT OFF)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

################################################################################

### Configuration
set(CITY3D_ROOT         ${CMAKE_CURRENT_LIST_DIR})
set(POLYFIT_INCLUDE_DIR  ${CITY3D_ROOT}  ${CMAKE_CURRENT_BINARY_DIR})
set(POLYFIT_SOURCE_DIR   ${CITY3D_ROOT})

set(3RD_PARTY_DIR   ${CITY3D_ROOT}/3rd_party)
set(POLYFIT_qglviewer_DIR           ${3RD_PARTY_DIR}/3rd_QGLViewer)
set(POLYFIT_scip_DIR                ${3RD_PARTY_DIR}/3rd_scip)
set(POLYFIT_soplex_DIR              ${3RD_PARTY_DIR}/3rd_soplex)
set(POLYFIT_glew_DIR                ${3RD_PARTY_DIR}/3rd_glew)
set(POLYFIT_eigen_DIR               ${3RD_PARTY_DIR}/3rd_eigen-3.3.7)

### conditionally compile certain modules depending on libraries found on the system
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

if (POLICY CMP0167)
        cmake_policy(SET CMP0167 NEW)
endif ()
find_package(Boost REQUIRED)
if (Boost_FOUND)
        message(STATUS "Found Boost-${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_BUGFIX_VERSION}")
endif ()

################################################################################

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/lib CACHE
    PATH "Directory where all the .lib files are dumped." FORCE)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/bin CACHE
    PATH "Directory where .exe and .dll files are dumped." FORCE)

################################################################################

add_subdirectory(3rd_party/3rd_scip)
add_subdirectory(3rd_party/3rd_soplex)
add_subdirectory(3rd_party/3rd_QGLViewer)
add_subdirectory(3rd_party/3rd_glew)
add_subdirectory(3rd_party/3rd_rply)
add_subdirectory(3rd_party/3rd_LAStools)

add_subdirectory(basic)
add_subdirectory(math)
add_subdirectory(model)
add_subdirectory(renderer)
add_subdirectory(method)

add_subdirectory(CLI_Example_1)  # command line example
add_subdirectory(CLI_Example_2)  # command line example
add_subdirectory(City3D)         # demo with GUI


