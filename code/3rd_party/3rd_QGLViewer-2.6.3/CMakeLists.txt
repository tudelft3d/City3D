cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

include(../../cmake/UseQt5.cmake)
if(NOT QT5_FOUND)
    message(WARNING "${PROJECT_NAME} requires Qt but Qt was not found. You can set 'Qt5_DIR' to the "
            "directory containing 'Qt5Config.cmake' or 'qt5-config.cmake' in 'CMakeCache.txt'. "
            "Optionally, you can set the Qt5 root directory 'QT5_ROOT_PATH' to the directory "
            "containing the 'bin' folder.")
    return()
endif()

find_package(Qt5 COMPONENTS Xml)

set(qglviewer_HEADERS
    camera.h
    config.h
    constraint.h
    domUtils.h
    frame.h
    ImageInterface.h
    keyFrameInterpolator.h
    manipulatedCameraFrame.h
    manipulatedFrame.h
    mouseGrabber.h
    qglviewer.h
    quaternion.h
    vec.h
    VRender/AxisAlignedBox.h
    VRender/Exporter.h
    VRender/gpc.h
    VRender/NVector3.h
    VRender/Optimizer.h
    VRender/ParserGL.h
    VRender/Primitive.h
    VRender/PrimitivePositioning.h
    VRender/SortMethod.h
    VRender/Types.h
    VRender/Vector2.h
    VRender/Vector3.h
    VRender/VRender.h
    )

set(qglviewer_SOURCES
    camera.cpp
    constraint.cpp
    frame.cpp
    ImageInterface.cpp
    keyFrameInterpolator.cpp
    manipulatedCameraFrame.cpp
    manipulatedFrame.cpp
    mouseGrabber.cpp
    qglviewer.cpp
    quaternion.cpp
    saveSnapshot.cpp
    vec.cpp
    VRender/BackFaceCullingOptimizer.cpp
    VRender/BSPSortMethod.cpp
    VRender/EPSExporter.cpp
    VRender/Exporter.cpp
    VRender/FIGExporter.cpp
    VRender/gpc.cpp
    VRender/NVector3.cpp
    VRender/ParserGL.cpp
    VRender/Primitive.cpp
    VRender/PrimitivePositioning.cpp
    VRender/TopologicalSortMethod.cpp
    VRender/Vector2.cpp
    VRender/Vector3.cpp
    VRender/VisibilityOptimizer.cpp
    VRender/VRender.cpp
    )


add_library(${PROJECT_NAME} STATIC ${qglviewer_SOURCES} ${qglviewer_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES
                      FOLDER "3rd_party")


target_include_directories(${PROJECT_NAME} PRIVATE
    ${POLYFIT_qglviewer_DIR}/shared
    ${POLYFIT_qglviewer_DIR}/bfp
    ${POLYFIT_qglviewer_DIR}/bfp/bfp_LUSOL
    ${POLYFIT_qglviewer_DIR}/bfp/bfp_LUSOL/LUSOL
    ${POLYFIT_qglviewer_DIR}/colamd
    )


if(MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
            _CRT_SECURE_NO_WARNINGS
            _CRT_SECURE_NO_DEPRECATE
            QGLVIEWER_STATIC
        )
endif()


target_link_libraries( ${PROJECT_NAME} Qt5::Core )
target_link_libraries( ${PROJECT_NAME} Qt5::Gui )
target_link_libraries( ${PROJECT_NAME} Qt5::Widgets )
target_link_libraries( ${PROJECT_NAME} Qt5::OpenGL )
target_link_libraries( ${PROJECT_NAME} Qt5::Xml )

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

target_link_libraries( ${PROJECT_NAME} ${OPENGL_LIBRARIES} )
