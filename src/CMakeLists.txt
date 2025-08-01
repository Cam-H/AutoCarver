
qt_add_library(AutoCarverLib

        core/Timer.cpp
        core/Sculpture.cpp
        core/Scene.cpp

        core/SculptProcess.cpp

        fileIO/MeshHandler.cpp
        fileIO/Serializable.cpp
        fileIO/Serializer.cpp


        geometry/poly/Polygon.cpp
        geometry/poly/Profile.cpp

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

        robot/trajectory/Trajectory.cpp
        robot/planning/PathPlanner.cpp


        physics/RigidBody.cpp
        physics/CompositeBody.cpp
        physics/Constraint.cpp


        renderer/SceneWidget.cpp
        renderer/LineChartWidget.cpp

        renderer/RenderCapture.cpp

        renderer/RenderGeometry.cpp
        renderer/UiLoader.cpp

        renderer/Camera.cpp
        renderer/EdgeDetect.cpp

        renderer/PolygonWidget.cpp


        renderer/Colors.h


        geometry/Axis3D.cpp
        geometry/curves/Interpolator.cpp
        robot/trajectory/TOPPTrajectory.cpp
        robot/trajectory/SimpleTrajectory.cpp
        robot/trajectory/Waypoint.cpp
        robot/Pose.cpp
        robot/trajectory/CompositeTrajectory.cpp
        geometry/curves/CubicSpline.cpp
        geometry/curves/PiecewisePolyPath.cpp
        robot/trajectory/TOPP.cpp
        robot/trajectory/CartesianTrajectory.cpp
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