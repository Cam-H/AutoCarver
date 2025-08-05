# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(raycast LANGUAGES CXX)

qt_add_executable(raycast
        main.cpp
        main.ui
        )

target_link_libraries(raycast PRIVATE AutoCarverLib)

install(TARGETS raycast
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )
