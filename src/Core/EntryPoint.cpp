#include <QApplication>
#include <QPushButton>

#include <QSurfaceFormat>
#include <QLabel>

#ifndef QT_NO_OPENGL
#include "../Widgets/SceneViewWidget.h"
#endif

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("Auto Carver");
//    QApplication::setApplicationVersion("0.1");

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(format);

#ifndef QT_NO_OPENGL
    SceneViewWidget widget;
    widget.show();
#else
    QLabel note("OpenGL Support required");
    note.setWindowTitle("Error!");
    note.setWindowOpacity(0.4);
    note.show();
#endif

    return QApplication::exec();
}
