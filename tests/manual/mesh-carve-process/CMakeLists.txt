# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(meshcarving LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets UiTools Widgets Test)

qt_add_executable(meshcarving
        main.cpp
        )

target_link_libraries(meshcarving PRIVATE AutoCarverLib)

install(TARGETS meshcarving
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )