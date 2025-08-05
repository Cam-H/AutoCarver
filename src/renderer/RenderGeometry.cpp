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

    std::vector<std::reference_wrapper<const std::vector<glm::dvec3>>> attribs = attributes(mesh);
    for (const std::vector<glm::dvec3>& attribute : attribs) {
        if (attribute.empty()) throw std::runtime_error("[RenderGeometry] Attribute data not populated! Can not prepare OpenGL buffers");
    }

    m_stride = 3 * attribs.size() * sizeof(double);

    // Generate buffer for vertices
    uint32_t vertexCount = m_indexVertices ? mesh->vertexCount() : m_indexCount;
    auto *data = new double[vertexCount * m_stride];
    auto *ptr = data;

    if (m_indexVertices) {
        for (uint32_t i = 0; i < vertexCount; i++) {
            for (const std::vector<glm::dvec3>& attribute : attribs) {
                const glm::dvec3& value = attribute[i];
                *ptr++ = value.x;
                *ptr++ = value.y;
                *ptr++ = value.z;
            }
        }
    } else {
        for (uint32_t i = 0; i < mesh->faceCount(); i++) { // Every face
            auto [start, count] = mesh->faces().triangleLookup(i);
            for (uint32_t j = 0; j < count; j++) { // Every triangle in each face
                const TriIndex& triangle = mesh->faces().triangles()[start + j];

                for (uint32_t k = 0; k < 3; k++) { // Every vertex in each triangle
                    const glm::dvec3& vertex = attribs[0].get()[triangle[k]];

                    *ptr++ = vertex.x;
                    *ptr++ = vertex.y;
                    *ptr++ = vertex.z;

                    for (uint32_t m = 1; m < attribs.size(); m++) { // Every attribute in each vertex
                        const glm::dvec3& value = attribs[m].get()[i];
                        *ptr++ = value.x;
                        *ptr++ = value.y;
                        *ptr++ = value.z;

                    }
                }
            }
        }
    }

    return data;
}

std::vector<std::reference_wrapper<const std::vector<glm::dvec3>>> RenderGeometry::attributes(const std::shared_ptr<Mesh>& mesh)
{
    switch (m_format) {
        case Format::VERTEX:
            return { mesh->vertices().vertices() };
        case Format::VERTEX_NORMAL:
            if (m_indexVertices) return { mesh->vertices().vertices(), mesh->vertexNormals().vertices() };
            else return { mesh->vertices().vertices(), mesh->faces().normals() };
        case Format::VERTEX_COLOR:
            if (m_indexVertices) return { mesh->vertices().vertices(), mesh->vertexColors() };
            else return { mesh->vertices().vertices(), mesh->faces().colors() };
        case Format::VERTEX_NORMAL_COLOR:
            if (m_indexVertices) return { mesh->vertices().vertices(), mesh->vertexNormals().vertices(), mesh->vertexColors() };
            else return { mesh->vertices().vertices(), mesh->faces().normals(), mesh->faces().colors() };
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
        program->setAttributeBuffer(location, GL_DOUBLE, offset, 3, m_stride);
        offset += 3 * sizeof(double);
    }
}
