# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(robotpathing LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets UiTools Widgets Test)

qt_standard_project_setup()

qt_add_executable(robotpathing
        main.cpp
        main.ui
        )

set_target_properties(robotpathing PROPERTIES
        #        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )

target_link_libraries(robotpathing PRIVATE
        Qt::Core
        Qt::Gui
        Qt::UiTools
        Qt::OpenGL
        Qt::OpenGLWidgets
        Qt::Widgets
        Qt::Test
        libassimp-6d
        AutoCarverLib
        )

# Resources:
#set(shaders_resource_files
#        "fshader.glsl"
#        "vshader.glsl"
#        )
#
#qt6_add_resources(cube "shaders"
#        PREFIX
#        "/"
#        FILES
#        ${shaders_resource_files}
#        )
#
#set(textures_resource_files
#        "cube.png"
#        )
#
#qt6_add_resources(cube "textures"
#        PREFIX
#        "/"
#        FILES
#        ${textures_resource_files}
#        )

install(TARGETS robotpathing
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

qt_generate_deploy_app_script(
        TARGET robotpathing
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})

