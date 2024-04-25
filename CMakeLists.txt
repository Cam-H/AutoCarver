# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(AutoCarver LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS 3DCore 3DExtras 3DInput 3DRender Core Gui OpenGL OpenGLWidgets Widgets)

qt_standard_project_setup()

qt_add_executable(AutoCarver
        src/core/EntryPoint.cpp
        src/widgets/SceneViewWidget.h
        src/widgets/SceneViewWidget.cpp
        src/renderer/VertexArray.h
        src/renderer/VertexArray.cpp
        src/renderer/MonitorCameraController.h
        src/renderer/MonitorCameraController.cpp
        src/geometry/Tesselation.h
        src/geometry/Tesselation.cpp
        src/fileIO/FileHandler.h
        src/fileIO/FileHandler.cpp
        src/fileIO/DataHandler.h
        src/fileIO/DataHandler.cpp
        )

set_target_properties(AutoCarver PROPERTIES
        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )

target_link_libraries(AutoCarver PRIVATE
        Qt::3DCore
        Qt::3DExtras
        Qt::3DInput
        Qt::3DRender
        Qt6::Core
        Qt6::Gui
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        )

# Resources:
set(shaders_resource_files
        "res/fshader.glsl"
        "res/vshader.glsl"
        )

qt6_add_resources(AutoCarver "res/shaders"
        PREFIX
        "/"
        FILES
        ${shaders_resource_files}
        )

#set(textures_resource_files
#        "cube.png"
#        )
#
#qt6_add_resources(AutoCarver "textures"
#        PREFIX
#        "/"
#        FILES
#        ${textures_resource_files}
#        )

install(TARGETS AutoCarver
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

qt_generate_deploy_app_script(
        TARGET AutoCarver
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
