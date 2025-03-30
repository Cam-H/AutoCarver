//
// Created by Cam on 2025-03-18.
//

#ifndef AUTOCARVER_RENDERGEOMETRY_H
#define AUTOCARVER_RENDERGEOMETRY_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

#include "geometry/Mesh.h"

class RenderGeometry : protected QOpenGLFunctions
{
public:

    enum class Format {
        VERTEX = 0, VERTEX_NORMAL, VERTEX_COLOR, VERTEX_NORMAL_COLOR
    };

    explicit RenderGeometry(const std::shared_ptr<Mesh>& mesh, RenderGeometry::Format format = RenderGeometry::Format::VERTEX, bool indexVertices = false);
    virtual ~RenderGeometry();

    void draw(QOpenGLShaderProgram *program);

private:

    void drawArray(QOpenGLShaderProgram *program);
    void drawIndexed(QOpenGLShaderProgram *program);

    void bindAttributes(QOpenGLShaderProgram *program);

    void initialize(const std::shared_ptr<Mesh>& mesh);

    void* vertexData(const std::shared_ptr<Mesh>& mesh);
    std::vector<const float*> attributes(const std::shared_ptr<Mesh>& mesh);

    QOpenGLBuffer m_arrayBuf;
    QOpenGLBuffer m_indexBuf;

    RenderGeometry::Format m_format;
    bool m_indexVertices;

    int m_indexCount;
    int m_stride;

    const char* VERTEX_NAME = "a_position";
    const char* NORMAL_NAME = "a_normal";
    const char* COLOR_NAME = "a_color";

};


#endif //AUTOCARVER_RENDERGEOMETRY_H
