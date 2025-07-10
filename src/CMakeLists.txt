
qt_add_library(AutoCarverLib

        core/Timer.h
        core/Timer.cpp
        core/Sculpture.h
        core/Sculpture.cpp
        core/Scene.h
        core/Scene.cpp

        core/SculptProcess.cpp
        core/SculptProcess.h

        fileIO/MeshHandler.h
        fileIO/MeshHandler.cpp
        fileIO/Serializable.cpp
        fileIO/Serializable.h
        fileIO/Serializer.cpp
        fileIO/Serializer.h


        physics/RigidBody.h
        physics/RigidBody.cpp
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
        geometry/FaceArray.cpp
        geometry/FaceArray.h

        geometry/Transformable.cpp
        geometry/Transformable.h
        geometry/Simplex.cpp
        geometry/Simplex.h
        geometry/EPA.cpp
        geometry/EPA.h
        geometry/Circle.cpp
        geometry/Circle.h
        geometry/Sphere.cpp
        geometry/Sphere.h

        geometry/poly/Polygon.h
        geometry/poly/Polygon.cpp
        geometry/poly/Profile.h
        geometry/poly/Profile.cpp

        robot/KinematicChain.cpp
        robot/KinematicChain.h
        robot/ArticulatedWrist.cpp
        robot/ArticulatedWrist.h
        robot/Joint.cpp
        robot/Joint.h
        robot/Robot.cpp
        robot/Robot.h

        robot/planning/Trajectory.cpp
        robot/planning/Trajectory.h
        robot/planning/PathPlanner.cpp
        robot/planning/PathPlanner.h


        physics/Constraint.cpp
        physics/Constraint.h


        renderer/SceneWidget.cpp
        renderer/SceneWidget.h
        renderer/LineChartWidget.cpp
        renderer/LineChartWidget.h

        renderer/RenderCapture.cpp
        renderer/RenderCapture.h

        renderer/RenderGeometry.cpp
        renderer/RenderGeometry.h
        renderer/UiLoader.cpp
        renderer/UiLoader.h

        renderer/Camera.cpp renderer/Camera.h
        renderer/EdgeDetect.cpp
        renderer/EdgeDetect.h

        renderer/PolygonWidget.cpp renderer/PolygonWidget.h physics/CompositeBody.cpp physics/CompositeBody.h geometry/Octree.cpp geometry/Octree.h)

target_link_libraries(AutoCarverLib PRIVATE
        Qt::Core
        Qt::Gui
        Qt::UiTools
        Qt::OpenGL
        Qt::OpenGLWidgets
        Qt::Widgets
        libassimp-6d
        )

qt_add_executable(AutoCarver
        core/EntryPoint.cpp)

target_link_libraries(AutoCarver PRIVATE
        Qt::Core
        Qt::Gui
        Qt::UiTools
        Qt::OpenGL
        Qt::OpenGLWidgets
        Qt::Widgets
        libassimp-6d
        AutoCarverLib
        )

set_target_properties(AutoCarver PROPERTIES
#        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )