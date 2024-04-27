
qt_add_executable(AutoCarver
        core/EntryPoint.cpp
        core/Scene.cpp

        widgets/SceneViewWidget.cpp

        renderer/VertexArray.cpp
        renderer/MonitorCameraController.cpp

        geometry/Triangle.cpp
        geometry/Tesselation.cpp
        geometry/GeometryBuilder.cpp

#        fileIO/FileHandler.cpp
#        fileIO/DataHandler.cpp
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

set_target_properties(AutoCarver PROPERTIES
        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )