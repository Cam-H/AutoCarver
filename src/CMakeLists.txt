
add_library(AutoCarverLib
        core/Timer.cpp
        core/Sculpture.cpp
        core/Debris.cpp
        core/Scene.cpp

        process/SculptProcess.cpp
        process/ProcessPlanner.cpp
        process/ProcessConfiguration.cpp
        process/Sequence.cpp

        fileIO/MeshHandler.cpp
        fileIO/Serializable.cpp
        fileIO/Serializer.cpp


        geometry/poly/Polygon.cpp
        geometry/poly/Profile.cpp
        geometry/poly/SectionOperation.cpp

        geometry/primitives/AABB.cpp
        geometry/primitives/Circle.cpp
        geometry/primitives/ConvexHull.cpp
        geometry/primitives/Plane.cpp
        geometry/primitives/Ray.cpp
        geometry/primitives/Sphere.cpp
        geometry/primitives/Triangle.cpp

        geometry/collision/Collision.cpp
        geometry/collision/EPA.cpp

        geometry/Octree.cpp

        geometry/Mesh.cpp
        geometry/MeshBuilder.cpp

        geometry/VertexArray.cpp
        geometry/FaceArray.cpp

        geometry/Transformable.cpp
        geometry/collision/Simplex.cpp

        robot/KinematicChain.cpp
        robot/ArticulatedWrist.cpp
        robot/Joint.cpp
        robot/Robot.cpp

        robot/planning/PathPlanner.cpp

        physics/Body.cpp
        physics/RigidBody.cpp
        physics/CompositeBody.cpp
        physics/Constraint.cpp


        renderer/RenderBuffer.cpp
        renderer/SceneWidget.cpp
        renderer/LineChartWidget.cpp

        renderer/RenderCapture.cpp

        renderer/RenderGeometry.cpp
        renderer/UiLoader.cpp

        renderer/Camera.cpp
        renderer/EdgeDetect.cpp

        renderer/PolygonWidget.cpp


        renderer/Colors.h

        geometry/curves/Interpolator.cpp

        geometry/curves/CubicSpline.cpp
        geometry/curves/PiecewisePolyPath.cpp

        geometry/Axis3D.cpp

        robot/Pose.cpp

        robot/trajectory/Waypoint.cpp

        robot/trajectory/Trajectory.cpp
        robot/trajectory/SimpleTrajectory.cpp
        robot/trajectory/HoldPosition.cpp

        robot/trajectory/TOPP.cpp
        robot/trajectory/TOPPTrajectory.cpp

        robot/trajectory/CartesianTrajectory.cpp

        robot/trajectory/CompositeTrajectory.cpp

        )

set_target_properties(AutoCarverLib PROPERTIES
        AUTOMOC ON
        AUTOUIC ON
        )

target_link_libraries(AutoCarverLib PUBLIC
        Qt::Core
        Qt::Gui
        Qt::UiTools
        Qt::Widgets
        Qt::OpenGL
        Qt::OpenGLWidgets
        libassimp-6d
        )

add_executable(AutoCarver core/EntryPoint.cpp)

target_link_libraries(AutoCarver PRIVATE AutoCarverLib)