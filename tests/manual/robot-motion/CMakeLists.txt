# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(robotmotion LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core 3DCore 3DRender Test)

qt_standard_project_setup()

qt_add_executable(robotmotion
        main.cpp
        mycapture.h)

set_target_properties(robotmotion PROPERTIES
        #        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )

target_link_libraries(robotmotion PRIVATE
        Qt::3DCore
        Qt::3DExtras
        Qt::3DRender
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::Test
        reactphysics3d
        libassimp
        AutoCarverLib
        )

install(TARGETS robotmotion
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

qt_generate_deploy_app_script(
        TARGET robotmotion
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})

