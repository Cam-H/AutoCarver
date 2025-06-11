//
// Created by Cam on 2025-03-18.
//

#include "RenderGeometry.h"

#include <QVector2D>
#include <QVector3D>

#include <iostream>

RenderGeometry::RenderGeometry(const std::shared_ptr<Mesh>& mesh, RenderGeometry::Format format, bool indexVertices)
    : m_indexBuf(QOpenGLBuffer::IndexBuffer)
    , m_indexCount(3 * mesh->triangleCount())
    , m_format(format)
    , m_indexVertices(indexVertices)
    , m_stride(0)
{
    initializeOpenGLFunctions();

    // Generate VBOs
    m_arrayBuf.create();
    if (m_indexVertices) m_indexBuf.create();

    // Initializes cube geometry and transfers it to VBOs
    initialize(mesh);
}

void RenderGeometry::initialize(const std::shared_ptr<Mesh>& mesh)
{
    if (m_indexVertices) {
        m_arrayBuf.bind();
        auto *data = vertexData(mesh);
        m_arrayBuf.allocate(data, mesh->vertexCount() * m_stride);

        m_indexBuf.bind();
        m_indexBuf.allocate(mesh->indices(), m_indexCount * sizeof(uint32_t));
    } else {
        m_arrayBuf.bind();
        auto *data = vertexData(mesh);
        m_arrayBuf.allocate(data, m_indexCount * m_stride);
    }
}

void* RenderGeometry::vertexData(const std::shared_ptr<Mesh>& mesh)
{

    std::vector<std::reference_wrapper<const VertexArray>> attribs = attributes(mesh);

    m_stride = 3 * attribs.size() * sizeof(float);

    // Generate buffer for vertices
    uint32_t vertexCount = m_indexVertices ? mesh->vertexCount() : m_indexCount;
    auto *data = new float[vertexCount * m_stride];
    auto *ptr = data;

    if (m_indexVertices) {
        for (uint32_t i = 0; i < vertexCount; i++) {
            for (const VertexArray& attribute : attribs) {
                const glm::vec3& value = attribute.vertices()[i];
                *ptr++ = value.x;
                *ptr++ = value.y;
                *ptr++ = value.z;
            }
        }
    } else {
        const uint32_t* triPtr = mesh->indices();
        for (uint32_t i = 0; i < mesh->faceCount(); i++) {
            uint32_t size = 3 * (mesh->faces().faceSizes()[i] - 2);
            for (uint32_t j = 0; j < size; j++) {
                const glm::vec3& vertex = attribs[0].get().vertices()[*triPtr++];

                *ptr++ = vertex.x;
                *ptr++ = vertex.y;
                *ptr++ = vertex.z;

                for (uint32_t k = 1; k < attribs.size(); k++) {
                    const glm::vec3& value = attribs[k].get().vertices()[i];
                    *ptr++ = value.x;
                    *ptr++ = value.y;
                    *ptr++ = value.z;
                }
            }
        }
    }

    return data;
}

std::vector<std::reference_wrapper<const VertexArray>> RenderGeometry::attributes(const std::shared_ptr<Mesh>& mesh)
{
    switch (m_format) {
        case Format::VERTEX:
            return { mesh->vertices() };
        case Format::VERTEX_NORMAL:
            if (m_indexVertices) return { mesh->vertices(), mesh->vertexNormals() };
            else return { mesh->vertices(), mesh->faceNormals() };
        case Format::VERTEX_COLOR:
            return { mesh->vertices(), mesh->colors() };
        case Format::VERTEX_NORMAL_COLOR:
            if (m_indexVertices) return { mesh->vertices(), mesh->vertexNormals(), mesh->colors() };
            else return { mesh->vertices(), mesh->faceNormals(), mesh->colors() };
    }

    return {};
}

RenderGeometry::~RenderGeometry()
{
    m_arrayBuf.destroy();
    m_indexBuf.destroy();
}


void RenderGeometry::draw(QOpenGLShaderProgram *program)
{
    if (m_indexVertices) drawIndexed(program);
    else drawArray(program);
}

void RenderGeometry::drawArray(QOpenGLShaderProgram *program)
{
    m_arrayBuf.bind();

    bindAttributes(program);

    glDrawArrays(GL_TRIANGLES, 0, m_indexCount);

}
void RenderGeometry::drawIndexed(QOpenGLShaderProgram *program)
{
    // Tell OpenGL which VBOs to use
    m_arrayBuf.bind();
    m_indexBuf.bind();

    bindAttributes(program);

    // Draw geometry using indices from VBO 1
    glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, nullptr);
}

// Tell OpenGL programmable pipeline how to locate vertex position data
void RenderGeometry::bindAttributes(QOpenGLShaderProgram *program)
{
    std::vector<int> locations { program->attributeLocation(VERTEX_NAME) };

    switch (m_format) {
        case Format::VERTEX:
            break;
        case Format::VERTEX_NORMAL:
            locations.push_back(program->attributeLocation(NORMAL_NAME));
            break;
        case Format::VERTEX_COLOR:
            locations.push_back(program->attributeLocation(COLOR_NAME));
            break;
        case Format::VERTEX_NORMAL_COLOR:
            locations.push_back(program->attributeLocation(NORMAL_NAME));
            locations.push_back(program->attributeLocation(COLOR_NAME));
            break;
    }

    int offset = 0;
    for (int location : locations) {
        program->enableAttributeArray(location);
        program->setAttributeBuffer(location, GL_FLOAT, offset, 3, m_stride);
        offset += 3 * sizeof(float);
    }
}
