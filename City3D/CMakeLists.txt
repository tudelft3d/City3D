cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

include(../cmake/UseQt5.cmake)
if (NOT QT5_FOUND)
    message(WARNING "${PROJECT_NAME} requires Qt but Qt was not found. You can set 'Qt5_DIR' to the "
            "directory containing 'Qt5Config.cmake' or 'qt5-config.cmake' in 'CMakeCache.txt'. "
            "Optionally, you can set the Qt5 root directory 'QT5_ROOT_PATH' to the directory "
            "containing the 'bin' folder.")
    return()
endif ()

find_package(Qt5 COMPONENTS Xml)

set(${PROJECT_NAME}_HEADERS
        main_window.h
        paint_canvas.h
        dlg/wgt_render.h
        dlg/weight_panel_click.h
        dlg/weight_panel_manual.h
        )

set(${PROJECT_NAME}_SOURCES
        main_window.cpp
        main.cpp
        paint_canvas.cpp
        dlg/wgt_render.cpp
        dlg/weight_panel_click.cpp
        dlg/weight_panel_manual.cpp
        )

set(${PROJECT_NAME}_FORMS
        main_window.ui
        dlg/wgt_render.ui
        dlg/weight_panel_click.ui
        dlg/weight_panel_manual.ui
        )

set(${PROJECT_NAME}_RESOURCES
        Resources/FootPrint.qrc
        Resources/FootPrint.rc
        )

if (APPLE)
    add_executable(${PROJECT_NAME} MACOSX_BUNDLE ${${PROJECT_NAME}_HEADERS} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_FORMS} ${${PROJECT_NAME}_RESOURCES})
else ()
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_HEADERS} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_FORMS} ${${PROJECT_NAME}_RESOURCES})
endif ()


set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "LOD2")


# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_DEPRECATED_WARNINGS)


target_link_libraries(${PROJECT_NAME} Qt5::Core)
target_link_libraries(${PROJECT_NAME} Qt5::Gui)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} Qt5::OpenGL)
target_link_libraries(${PROJECT_NAME} Qt5::Xml)

target_link_libraries(${PROJECT_NAME} basic math model method renderer 3rd_glew 3rd_QGLViewer-2.6.3)

include(../cmake/FindGUROBI.cmake)
if (GUROBI_FOUND)
    message(STATUS "Gurobi include dir: " ${GUROBI_INCLUDE_DIRS})
    message(STATUS "------Gurobi libraries----------: " ${GUROBI_LIBRARIES})
    target_link_libraries(${PROJECT_NAME} ${GUROBI_LIBRARIES})
endif()


# Note: On macOS, high-DPI support is enabled by settings in the Info.plist file. Make sure they are present.
# For more information, see: http://doc.qt.io/qt-5/highdpi.html
# Enable high DPI support by using my Info.plist file in which I added NSHighResolutionCapable to be true.
if (APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Mac/LOD2.plist)
endif ()
