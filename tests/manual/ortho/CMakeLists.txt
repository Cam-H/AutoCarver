# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(ortho LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets UiTools Widgets Test)

qt_add_executable(ortho
        main.cpp
        main.ui
        )

target_link_libraries(ortho PRIVATE AutoCarverLib)

install(TARGETS ortho
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )
