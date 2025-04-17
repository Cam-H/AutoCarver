# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)

project(AutoCarver VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(-DASSIMP_WARNINGS_AS_ERRORS OFF)

find_package(Qt6 REQUIRED COMPONENTS 3DCore 3DExtras 3DInput 3DRender Core Gui UiTools OpenGL OpenGLWidgets Widgets)

qt_standard_project_setup()


#find_library(ASSIMP_LIBRARY assimp PATHS ./extern/assimp/bin)
link_directories(./extern/assimp/bin ./extern/assimp/lib)
link_directories(./extern/mcut/bin ./extern/mcut/lib)
include_directories(./extern/glm)

include_directories(./extern/assimp/include)
include_directories(./extern/mcut/include)
include_directories(extern/reactphysics3d/include)
include_directories(./extern/glm)

#add_library()
add_subdirectory(extern/reactphysics3d)

#set(MCUT_BUILD_WITH_COMPUTE_HELPER_THREADPOOL 0)
#add_subdirectory(./extern/mcut)

#add_library(mcut SHARED ./extern/mcut/source/*.cpp)
#target_include_directories(mcut PRIVATE ./extern/mcut/include ./extern/mcut/source)

#set(ASSIMP_DIR ./extern/assimp/bin)
#message(${ASSIMP_DIR})
#link_directories(./extern/assimp/lib)
#add_subdirectory(./extern/assimp)

include_directories(src)
add_subdirectory(src)

set(TEST_SOURCE_DIRECTORY ${PROJECT_SOURCE_DIR})
add_subdirectory(tests)

