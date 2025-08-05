# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(collision LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets Widgets Test)


qt_add_executable(collision
        main.cpp
        ControlWidget.cpp
        ControlWidget.h)

target_link_libraries(collision PRIVATE AutoCarverLib)

install(TARGETS collision
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )