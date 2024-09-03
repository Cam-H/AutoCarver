
qt_add_library(AutoCarverLib
        core/Scene.h
        core/Scene.cpp

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
        geometry/Tesselation.h
        geometry/Tesselation.cpp
        geometry/GeometryBuilder.h
        geometry/GeometryBuilder.cpp
)

#target_include_directories(AutoCarverLib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

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
        )

qt_add_executable(AutoCarver
        core/EntryPoint.cpp


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
        AutoCarverLib
        )

set_target_properties(AutoCarver PROPERTIES
        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE TRUE
        )