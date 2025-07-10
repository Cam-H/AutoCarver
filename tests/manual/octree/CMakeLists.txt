# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(octree LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets UiTools Widgets Test)

qt_standard_project_setup()

qt_add_executable(octree
        main.cpp
        main.ui
        )

set_target_properties(octree PROPERTIES
        #        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )

target_link_libraries(octree PRIVATE
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

install(TARGETS octree
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

qt_generate_deploy_app_script(
        TARGET octree
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})

