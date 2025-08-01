
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


        geometry/poly/Polygon.h
        geometry/poly/Polygon.cpp
        geometry/poly/Profile.h
        geometry/poly/Profile.cpp

        geometry/primitives/AABB.cpp
        geometry/primitives/AABB.h
        geometry/primitives/Circle.cpp
        geometry/primitives/Circle.h
        geometry/primitives/ConvexHull.h
        geometry/primitives/ConvexHull.cpp
        geometry/primitives/Plane.cpp
        geometry/primitives/Plane.h
        geometry/primitives/Ray.cpp
        geometry/primitives/Ray.h
        geometry/primitives/Sphere.cpp
        geometry/primitives/Sphere.h
        geometry/primitives/Triangle.h
        geometry/primitives/Triangle.cpp

        geometry/collision/Collision.cpp
        geometry/collision/Collision.h
        geometry/collision/EPA.cpp
        geometry/collision/EPA.h

        geometry/Octree.cpp
        geometry/Octree.h

        geometry/Mesh.h
        geometry/Mesh.cpp
        geometry/MeshBuilder.h
        geometry/MeshBuilder.cpp

        geometry/VertexArray.h
        geometry/VertexArray.cpp
        geometry/FaceArray.cpp
        geometry/FaceArray.h

        geometry/Transformable.cpp
        geometry/Transformable.h
        geometry/collision/Simplex.cpp
        geometry/collision/Simplex.h

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


        physics/RigidBody.h
        physics/RigidBody.cpp
        physics/CompositeBody.cpp
        physics/CompositeBody.h
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

        renderer/Camera.cpp
        renderer/Camera.h
        renderer/EdgeDetect.cpp
        renderer/EdgeDetect.h

        renderer/PolygonWidget.cpp
        renderer/PolygonWidget.h


        renderer/Colors.h


        geometry/Axis3D.cpp
        geometry/Axis3D.h
        robot/planning/Interpolator.cpp
        robot/planning/Interpolator.h
        robot/planning/TOPPTrajectory.cpp
        robot/planning/TOPPTrajectory.h
        robot/planning/SimpleTrajectory.cpp
        robot/planning/SimpleTrajectory.h
        robot/planning/Waypoint.cpp
        robot/planning/Waypoint.h
        robot/planning/TrajectoryBuilder.cpp
        robot/planning/TrajectoryBuilder.h
        robot/Pose.cpp
        robot/Pose.h
        robot/planning/CompositeTrajectory.cpp
        robot/planning/CompositeTrajectory.h
        robot/planning/CubicSpline.cpp
        robot/planning/PiecewisePolyPath.cpp
        robot/planning/TOPP.cpp
        )

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