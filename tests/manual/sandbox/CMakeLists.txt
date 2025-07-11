# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(sandbox LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core 3DCore 3DRender Test)

qt_standard_project_setup()

qt_add_executable(sandbox
        sandbox.cpp
        )

set_target_properties(sandbox PROPERTIES
        #        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )

target_link_libraries(sandbox PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::Test
        AutoCarverLib
        )

install(TARGETS sandbox
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

qt_generate_deploy_app_script(
        TARGET sandbox
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})

