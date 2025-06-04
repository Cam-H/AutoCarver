# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(ortho LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets UiTools Widgets Test)

qt_standard_project_setup()

qt_add_executable(ortho
        main.cpp
        main.ui
        )

set_target_properties(ortho PROPERTIES
        #        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )

target_link_libraries(ortho PRIVATE
        Qt::3DCore
        Qt::3DExtras
        Qt::3DRender
        Qt6::Core
        Qt6::Gui
        Qt::UiTools
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        Qt6::Test
        libassimp
        AutoCarverLib
        )

install(TARGETS ortho
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

qt_generate_deploy_app_script(
        TARGET ortho
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})

