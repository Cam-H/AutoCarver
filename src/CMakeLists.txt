
qt_add_library(AutoCarverLib

        core/Timer.h
        core/Timer.cpp
        core/Sculpture.h
        core/Sculpture.cpp
        core/Scene.h
        core/Scene.cpp


        fileIO/MeshHandler.h
        fileIO/MeshHandler.cpp


        geometry/Body.h
        geometry/Body.cpp
        geometry/ConvexHull.h
        geometry/ConvexHull.cpp
        geometry/Mesh.h
        geometry/Mesh.cpp
        geometry/MeshBuilder.h
        geometry/MeshBuilder.cpp
        geometry/Triangle.h
        geometry/Triangle.cpp
        geometry/VertexArray.h
        geometry/VertexArray.cpp

        geometry/poly/Polygon.h
        geometry/poly/Polygon.cpp
        geometry/poly/CompositePolygon.h
        geometry/poly/CompositePolygon.cpp


        robot/KinematicChain.cpp
        robot/KinematicChain.h
        robot/ArticulatedWrist.cpp
        robot/ArticulatedWrist.h
        robot/Joint.cpp
        robot/Joint.h
        robot/Robot.cpp
        robot/Robot.h

        renderer/MonitorCameraController.h
        renderer/MonitorCameraController.cpp
        renderer/RenderEntity.h
        renderer/RenderEntity.cpp


        widgets/SceneViewWidget.h
        widgets/SceneViewWidget.cpp

        core/SculptProcess.cpp core/SculptProcess.h geometry/FaceArray.cpp geometry/FaceArray.h

        renderer/RenderCapture.cpp renderer/RenderCapture.h renderer/SceneWidget.cpp renderer/SceneWidget.h renderer/RenderGeometry.cpp renderer/RenderGeometry.h
        geometry/Simplex.cpp geometry/Simplex.h geometry/EPA.cpp geometry/EPA.h
        fileIO/Serializable.cpp fileIO/Serializable.h fileIO/Serializer.cpp fileIO/Serializer.h geometry/Sphere.cpp geometry/Sphere.h renderer/LineChartWidget.cpp renderer/LineChartWidget.h renderer/UiLoader.cpp renderer/UiLoader.h geometry/Transformable.cpp geometry/Transformable.h

        robot/planning/PathPlanner.cpp robot/planning/PathPlanner.h
        robot/planning/Trajectory.cpp robot/planning/Trajectory.h
        )

target_link_libraries(AutoCarverLib PRIVATE
        Qt::3DCore
        Qt::3DExtras
        Qt::3DInput
        Qt::3DRender
        Qt6::Core
        Qt6::Gui
        Qt::UiTools
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        libassimp
        libmcutd
        reactphysics3d
        )

qt_add_executable(AutoCarver
        core/EntryPoint.cpp
        )

target_link_libraries(AutoCarver PRIVATE
        Qt::3DCore
        Qt::3DExtras
        Qt::3DInput
        Qt::3DRender
        Qt6::Core
        Qt6::Gui
        Qt::UiTools
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        libassimp
        libmcutd
        reactphysics3d
        AutoCarverLib
        )

set_target_properties(AutoCarver PROPERTIES
#        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )