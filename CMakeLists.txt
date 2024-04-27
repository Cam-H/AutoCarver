# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)

project(AutoCarver VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS 3DCore 3DExtras 3DInput 3DRender Core Gui OpenGL OpenGLWidgets Widgets)

qt_standard_project_setup()

add_subdirectory(src)

add_subdirectory(tests)