//
// Created by cameronh on 23/04/24.
//

#ifndef AUTOCARVER_VERTEXARRAY_H
#define AUTOCARVER_VERTEXARRAY_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

class VertexArray: protected QOpenGLFunctions
{
public:
    VertexArray();
    ~VertexArray();

    void draw(QOpenGLShaderProgram *program);

private:

    QOpenGLBuffer vertexBuffer;
    QOpenGLBuffer indexBuffer;
};


#endif //AUTOCARVER_VERTEXARRAY_H
