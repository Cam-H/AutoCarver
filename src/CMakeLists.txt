
qt_add_library(AutoCarverLib
        core/Scene.h
        core/Scene.cpp
        core/Timer.h
        core/Timer.cpp

        fileIO/MeshLoader.h
        fileIO/MeshLoader.cpp

        widgets/SceneViewWidget.h
        widgets/SceneViewWidget.cpp

        renderer/VertexArray.h
        renderer/VertexArray.cpp
        renderer/MonitorCameraController.h
        renderer/MonitorCameraController.cpp

        geometry/Triangle.h
        geometry/Triangle.cpp
        geometry/Polygon.h
        geometry/Polygon.cpp
        geometry/Surface.h
        geometry/Surface.cpp
        geometry/Tesselation.h
        geometry/Tesselation.cpp
        geometry/GeometryBuilder.h
        geometry/GeometryBuilder.cpp
        geometry/Body.h
        geometry/Body.cpp
        )

target_link_libraries(AutoCarverLib PRIVATE
        Qt::3DCore
        Qt::3DExtras
        Qt::3DInput
        Qt::3DRender
        Qt6::Core
        Qt6::Gui
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        libassimp
        )

qt_add_executable(AutoCarver
        core/EntryPoint.cpp
        )

#target_include_directories(AutoCarver PUBLIC assimp)

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
        libassimp

        AutoCarverLib
        )

set_target_properties(AutoCarver PROPERTIES
#        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )