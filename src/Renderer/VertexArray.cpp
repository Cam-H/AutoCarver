//
// Created by cameronh on 23/04/24.
//

#include "VertexArray.h"

struct VertexData
{
    QVector3D position;
    QVector3D color;
};

VertexArray::VertexArray()
    : indexBuffer(QOpenGLBuffer::IndexBuffer)
{
    initializeOpenGLFunctions();

    vertexBuffer.create();
    indexBuffer.create();

    VertexData vertices[] = {
            // Vertex data for face 0
            {QVector3D(-1.0f, -1.0f,  1.0f), QVector3D(1, 0, 0)},  // v0
            {QVector3D( 1.0f, -1.0f,  1.0f), QVector3D(1, 0, 0)}, // v1
            {QVector3D(-1.0f,  1.0f,  1.0f), QVector3D(1, 0, 0)},  // v2
            {QVector3D( 1.0f,  1.0f,  1.0f), QVector3D(1, 0, 0)}, // v3

            // Vertex data for face 1
            {QVector3D( 1.0f, -1.0f,  1.0f), QVector3D(1, 0, 0)}, // v4
            {QVector3D( 1.0f, -1.0f, -1.0f), QVector3D(1, 0, 0)}, // v5
            {QVector3D( 1.0f,  1.0f,  1.0f), QVector3D(1, 0, 0)},  // v6
            {QVector3D( 1.0f,  1.0f, -1.0f), QVector3D(1, 0, 0)}, // v7

            // Vertex data for face 2
            {QVector3D( 1.0f, -1.0f, -1.0f), QVector3D(1, 0, 0)}, // v8
            {QVector3D(-1.0f, -1.0f, -1.0f), QVector3D(1, 0, 0)},  // v9
            {QVector3D( 1.0f,  1.0f, -1.0f), QVector3D(1, 0, 0)}, // v10
            {QVector3D(-1.0f,  1.0f, -1.0f), QVector3D(1, 0, 0)},  // v11

            // Vertex data for face 3
            {QVector3D(-1.0f, -1.0f, -1.0f), QVector3D(1, 0, 0)}, // v12
            {QVector3D(-1.0f, -1.0f,  1.0f), QVector3D(1, 0, 0)},  // v13
            {QVector3D(-1.0f,  1.0f, -1.0f), QVector3D(1, 0, 0)}, // v14
            {QVector3D(-1.0f,  1.0f,  1.0f), QVector3D(1, 0, 0)},  // v15

            // Vertex data for face 4
            {QVector3D(-1.0f, -1.0f, -1.0f), QVector3D(0, 0, 1)}, // v16
            {QVector3D( 1.0f, -1.0f, -1.0f), QVector3D(0, 0, 1)}, // v17
            {QVector3D(-1.0f, -1.0f,  1.0f), QVector3D(0, 0, 1)}, // v18
            {QVector3D( 1.0f, -1.0f,  1.0f), QVector3D(0, 0, 1)}, // v19

            // Vertex data for face 5
            {QVector3D(-1.0f,  1.0f,  1.0f), QVector3D(0, 1, 0)}, // v20
            {QVector3D( 1.0f,  1.0f,  1.0f), QVector3D(0, 1, 0)}, // v21
            {QVector3D(-1.0f,  1.0f, -1.0f), QVector3D(0, 1, 0)}, // v22
            {QVector3D( 1.0f,  1.0f, -1.0f), QVector3D(0, 1, 0)}  // v23
    };

    // Indices for drawing cube faces using triangle strips.
    // Triangle strips can be connected by duplicating indices
    // between the strips. If connecting strips have opposite
    // vertex order then last index of the first strip and first
    // index of the second strip needs to be duplicated. If
    // connecting strips have same vertex order then only last
    // index of the first strip needs to be duplicated.
    GLushort indices[] = {
            0,  1,  2,  3,  3,     // Face 0 - triangle strip ( v0,  v1,  v2,  v3)
            4,  4,  5,  6,  7,  7, // Face 1 - triangle strip ( v4,  v5,  v6,  v7)
            8,  8,  9, 10, 11, 11, // Face 2 - triangle strip ( v8,  v9, v10, v11)
            12, 12, 13, 14, 15, 15, // Face 3 - triangle strip (v12, v13, v14, v15)
            16, 16, 17, 18, 19, 19, // Face 4 - triangle strip (v16, v17, v18, v19)
            20, 20, 21, 22, 23      // Face 5 - triangle strip (v20, v21, v22, v23)
    };

//! [1]
    // Transfer vertex data to VBO 0
    vertexBuffer.bind();
    vertexBuffer.allocate(vertices, 24 * sizeof(VertexData));

    // Transfer index data to VBO 1
    indexBuffer.bind();
    indexBuffer.allocate(indices, 34 * sizeof(GLushort));
}

VertexArray::~VertexArray()
{
    vertexBuffer.destroy();
    indexBuffer.destroy();
}

void VertexArray::draw(QOpenGLShaderProgram *program)
{
    // Tell OpenGL which VBOs to use
    vertexBuffer.bind();
    indexBuffer.bind();

//    VertexData vertex[] = {QVector3D(2, 1, 1), QVector2D(0, 0)};
//    arrayBuf.write(0, vertex, sizeof(VertexData));

    // Offset for position
    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int colorLocation = program->attributeLocation("a_color");
    program->enableAttributeArray(colorLocation);
    program->setAttributeBuffer(colorLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Draw cube geometry using indices from VBO 1
    glDrawElements(GL_TRIANGLE_STRIP, 34, GL_UNSIGNED_SHORT, nullptr);
}