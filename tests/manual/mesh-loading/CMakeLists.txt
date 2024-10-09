# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(meshloading LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core 3DCore 3DRender Test)

qt_standard_project_setup()

qt_add_executable(meshloading
        main.cpp
        )

set_target_properties(meshloading PROPERTIES
        #        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )

target_link_libraries(meshloading PRIVATE
        Qt::3DCore
        Qt::3DExtras
        Qt::3DRender
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::Test
        libassimp
        AutoCarverLib
        )

install(TARGETS meshloading
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

qt_generate_deploy_app_script(
        TARGET meshloading
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})

