
qt_add_library(AutoCarverLib
        core/NotScene.h
        core/NotScene.cpp
        core/Timer.h
        core/Timer.cpp
        core/Scene.h
        core/Scene.cpp

        fileIO/MeshLoader.h
        fileIO/MeshLoader.cpp

        widgets/SceneViewWidget.h
        widgets/SceneViewWidget.cpp

        renderer/MonitorCameraController.h
        renderer/MonitorCameraController.cpp

        geometry/Triangle.h
        geometry/Triangle.cpp

        geometry/poly/Polygon.h
        geometry/poly/Polygon.cpp
        geometry/poly/ConvexPolygon.h
        geometry/poly/ConvexPolygon.cpp
        geometry/poly/CompositePolygon.h
        geometry/poly/CompositePolygon.cpp

        geometry/Surface.h
        geometry/Surface.cpp
        geometry/Tesselation.h
        geometry/Tesselation.cpp
        geometry/GeometryBuilder.h
        geometry/GeometryBuilder.cpp
        geometry/Body.h
        geometry/Body.cpp
        geometry/poly/ConvexPolygon.h
        geometry/poly/ConvexPolygon.cpp
        geometry/poly/CompositePolygon.h
        geometry/poly/CompositePolygon.cpp
        geometry/ConvexHull.h
        geometry/ConvexHull.cpp
        geometry/Mesh.h
        geometry/Mesh.cpp
        geometry/MeshBuilder.h
        geometry/MeshBuilder.cpp
        geometry/VertexArray.h
        geometry/VertexArray.cpp

        renderer/RenderEntity.h
        renderer/RenderEntity.cpp
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
        reactphysics3d
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
        reactphysics3d
        AutoCarverLib
        )

set_target_properties(AutoCarver PROPERTIES
#        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )