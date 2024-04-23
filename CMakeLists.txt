# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(AutoCarver LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets Widgets)

qt_standard_project_setup()

qt_add_executable(AutoCarver
        src/Core/EntryPoint.cpp
#        geometryengine.cpp geometryengine.h
#        main.cpp
#        mainwidget.cpp mainwidget.h
#        camera.cpp camera.h
        )

set_target_properties(AutoCarver PROPERTIES
        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )

target_link_libraries(AutoCarver PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        )

# Resources:
#set(shaders_resource_files
#        "fshader.glsl"
#        "vshader.glsl"
#        )

#qt6_add_resources(AutoCarver "shaders"
#        PREFIX
#        "/"
#        FILES
#        ${shaders_resource_files}
#        )

#set(textures_resource_files
#        "cube.png"
#        )
#
#qt6_add_resources(AutoCarver "textures"
#        PREFIX
#        "/"
#        FILES
#        ${textures_resource_files}
#        )

install(TARGETS AutoCarver
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

qt_generate_deploy_app_script(
        TARGET AutoCarver
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
