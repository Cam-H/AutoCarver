# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(collision LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets Widgets Test)

qt_standard_project_setup()

qt_add_executable(collision
        main.cpp
        ControlWidget.cpp
        ControlWidget.h)

set_target_properties(collision PROPERTIES
        #        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )

target_link_libraries(collision PRIVATE
        Qt::3DCore
        Qt::3DExtras
        Qt::3DRender
        Qt6::Core
        Qt6::Gui
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        Qt6::Test
        reactphysics3d
        libassimp
        AutoCarverLib
        )

install(TARGETS collision
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

qt_generate_deploy_app_script(
        TARGET collision
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})

