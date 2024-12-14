cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

include(../cmake/UseQt.cmake)
if (NOT Qt6_FOUND AND NOT Qt5_FOUND)
    message(WARNING "${PROJECT_NAME} requires Qt (Qt6 or Qt5), but Qt was not found. You can set 'Qt*_DIR' to the "
            "directory containing 'Qt*Config.cmake' or 'qt*-config.cmake' (* is 5 or 6) to specify the location of Qt.")
    return()
endif ()

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
        Resources/City3D.qrc
        Resources/City3D.rc
        Resources/City3D.icns
        )

if (APPLE)
    # set also the icon
    # This tells CMake where to find and install the ICON file
    set(${PROJECT_NAME}_ICON ${CMAKE_CURRENT_SOURCE_DIR}/Resources/City3D.icns)
    set_source_files_properties(${${PROJECT_NAME}_ICON} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

    add_executable( ${PROJECT_NAME} MACOSX_BUNDLE ${${PROJECT_NAME}_ICON} ${${PROJECT_NAME}_HEADERS} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_FORMS} ${${PROJECT_NAME}_RESOURCES} )

    set_source_files_properties(${${PROJECT_NAME}_ICON} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

    ## Note: On macOS, high-DPI support is enabled by settings in the Info.plist file. Make sure they are present.
    ## For more information, see: http://doc.qt.io/qt-5/highdpi.html
    ## Enable high DPI support by using my Info.plist file in which I added NSHighResolutionCapable to be true.
    set_target_properties(${PROJECT_NAME} PROPERTIES
            INSTALL_RPATH "@executable_path/../Frameworks"
            MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
            MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Mac/City3D.plist"
            MACOSX_BUNDLE_ICON_FILE City3D.icns
            )
elseif (MINGW)
    set(CMAKE_RC_COMPILER_INIT windres)
    enable_language(RC)
    set(CMAKE_RC_COMPILE_OBJECT "<CMAKE_RC_COMPILER> <FLAGS> -O coff <DEFINES> -i <SOURCE> -o <OBJECT>")
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_HEADERS} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_FORMS} ${${PROJECT_NAME}_RESOURCES})
else ()
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_HEADERS} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_FORMS} ${${PROJECT_NAME}_RESOURCES})
endif ()


set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "City3D")


# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_DEPRECATED_WARNINGS)

target_include_directories(${PROJECT_NAME} PRIVATE ${CITY3D_ROOT})

target_link_libraries(${PROJECT_NAME} ${QtLibs})
target_link_libraries(${PROJECT_NAME} basic math model method renderer 3rd_glew 3rd_QGLViewer)