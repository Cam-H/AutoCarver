//
// Created by Cam on 2024-11-09.
//

#include "RenderEntity.h"

// Rendering
#include <Qt3DCore/QAttribute>

#include <QTorusMesh>

#include "core/Timer.h"

RenderEntity::RenderEntity(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view)
    : Qt3DCore::QEntity(parent)
    , view(view)
    , m_transform(new Qt3DCore::QTransform())
{
    addComponent(m_transform);
}

void RenderEntity::show()
{
    setEnabled(true);
}
void RenderEntity::hide()
{
    setEnabled(false);
}

void RenderEntity::show(uint32_t idx)
{
    if (idx < m_renders.size()) m_renders[idx]->setEnabled(true);

    show(); // Make sure parent is also visible
}
void RenderEntity::hide(uint32_t idx)
{
    if (idx < m_renders.size()) m_renders[idx]->setEnabled(false);
}

void RenderEntity::setTranslation(QVector3D translation)
{
    m_transform->setTranslation(translation);
}

void RenderEntity::setRotation(QQuaternion rotation)
{
    m_transform->setRotation(rotation);
}

Qt3DCore::QTransform *RenderEntity::transformation()
{
    return m_transform;
}

void RenderEntity::add(const std::shared_ptr<Mesh>& mesh, Qt3DRender::QMaterial *material)
{
    if (material == nullptr) {
        material = new Qt3DExtras::QDiffuseSpecularMaterial();
        ((Qt3DExtras::QDiffuseSpecularMaterial*)material)->setDiffuse(QVariant(QColor(QRgb(0xa69929))));
//        ((Qt3DExtras::QDiffuseSpecularMaterial*)material)->setAlphaBlendingEnabled(true);
    }

    meshes.emplace_back(mesh, material);
}

void RenderEntity::generate()
{
    ScopedTimer("RE Gen");

    // Prevent materials from being deleted alongside the old entities (wll be reused)
    for (auto& mesh : meshes) {
        if (mesh.second != nullptr) {
            mesh.second->setParent((Qt3DCore::QNode*)nullptr);
        }
    }

    // Record state of entities
    std::vector<bool> enables;

    // Purge old entities
    for (Qt3DCore::QEntity *render : m_renders) {
        enables.push_back(render->isEnabled());
        render->deleteLater();
    }
    m_renders.clear();

    ScopedTimer timer("QRenderer creation");

    for (const auto& mesh : meshes) {
        auto entity = new Qt3DCore::QEntity(this);

        // Mesh transform
        entity->addComponent(new Qt3DCore::QTransform());


        // Mesh geometry
        auto renderer = new Qt3DRender::QGeometryRenderer(entity);

        if (mesh.first->faceCount() == mesh.first->triangleCount()) {
            entity->addComponent(mesh.second);

            renderer->setGeometry(indexedGeometry(mesh.first, renderer));
        } else {
            auto material = new Qt3DExtras::QPerVertexColorMaterial();
            entity->addComponent(material);

            renderer->setGeometry(faceGeometry(mesh.first, renderer));
        }

        entity->addComponent(renderer);

        m_renders.push_back(entity);
    }

    // Restore prior visibilities
    for (uint32_t i = 0; i < m_renders.size(); i++) {
        if (i >= enables.size()) break;

        m_renders[i]->setEnabled(enables[i]);
    }
}

Qt3DCore::QGeometry* RenderEntity::indexedGeometry(const std::shared_ptr<Mesh>& mesh, Qt3DCore::QNode *parent)
{
    auto geometry = new Qt3DCore::QGeometry(parent);

    uint32_t vertexCount = mesh->vertexCount();

    const float *attributes[] = {mesh->vertices(), mesh->normals()};
    uint8_t attributeCount = 2;

    const auto attributeNames = {
            Qt3DCore::QAttribute::defaultPositionAttributeName(),
            Qt3DCore::QAttribute::defaultNormalAttributeName()
    };

    uint32_t stride = 3 * attributeCount;// TODO - Low priority to support texture coordinate conversions
    size_t size = stride * vertexCount * sizeof(float);

    // Generate buffer for vertices
    QByteArray vertexData;
    vertexData.resize(size);

    auto vertexPtr = reinterpret_cast<float*>(vertexData.begin());

    for (uint32_t i = 0; i < vertexCount; i++) {
        for (uint8_t j = 0; j < attributeCount; j++) {
            *vertexPtr++ = attributes[j][3 * i];
            *vertexPtr++ = attributes[j][3 * i + 1];
            *vertexPtr++ = attributes[j][3 * i + 2];
        }
    }
    auto vertexBuffer = new Qt3DCore::QBuffer(geometry);
    vertexBuffer->setData(vertexData);

    // Prepare vertex attributes
    for (uint32_t i = 0; i < attributeCount; i++) {
        auto vertexAttribute = new Qt3DCore::QAttribute(geometry);
        vertexAttribute->setName(*(attributeNames.begin() + i));
        vertexAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
        vertexAttribute->setVertexBaseType(Qt3DCore::QAttribute::VertexBaseType::Float);
        vertexAttribute->setBuffer(vertexBuffer);
        vertexAttribute->setVertexSize(3);
        vertexAttribute->setByteStride(stride * sizeof(float));
        vertexAttribute->setByteOffset(3 * i * sizeof(float));
        vertexAttribute->setCount(vertexCount);

        geometry->addAttribute(vertexAttribute);
    }

    // Generate buffer for indices
    // TODO support uint16_t simultaneously
    size = 3 * mesh->triangleCount() * sizeof(uint32_t);
    QByteArray indexData;
    indexData.resize(size);

    auto indexPtr = reinterpret_cast<uint32_t*>(indexData.data());
    for (uint32_t i = 0; i < 3 * mesh->triangleCount(); i++) {
        *indexPtr++ = mesh->indices()[i];
    }

    auto indexBuffer = new Qt3DCore::QBuffer(geometry);
    indexBuffer->setData(indexData);

    // Prepare index attributes
    auto indexAttribute = new Qt3DCore::QAttribute(geometry);
    indexAttribute->setAttributeType(Qt3DCore::QAttribute::IndexAttribute);
    indexAttribute->setVertexBaseType(Qt3DCore::QAttribute::VertexBaseType::UnsignedInt);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(3 * mesh->triangleCount());

    geometry->addAttribute(indexAttribute);

    return geometry;
}

Qt3DCore::QGeometry* RenderEntity::faceGeometry(const std::shared_ptr<Mesh>& mesh, Qt3DCore::QNode *parent)
{
    auto geometry = new Qt3DCore::QGeometry(parent);

    uint32_t vertexCount = 3 * mesh->triangleCount();
    auto *vertices = new float[3 * vertexCount], *normals = new float[3 * vertexCount];

    mesh->directRepresentation(vertices, normals);

    float *attributes[3] = {vertices, normals, mesh->colors()};
    uint8_t attributeCount = 3;

    const auto attributeNames = {
            Qt3DCore::QAttribute::defaultPositionAttributeName(),
            Qt3DCore::QAttribute::defaultNormalAttributeName(),
            Qt3DCore::QAttribute::defaultColorAttributeName()
    };

    uint32_t stride = 3 * attributeCount;
    size_t size = stride * vertexCount * sizeof(float);

    // Generate buffer for vertices
    QByteArray vertexData;
    vertexData.resize(size);

    auto vertexPtr = reinterpret_cast<float*>(vertexData.begin());

    for (uint32_t i = 0; i < vertexCount; i++) {
        for (uint8_t j = 0; j < attributeCount; j++) {
            *vertexPtr++ = attributes[j][3 * i];
            *vertexPtr++ = attributes[j][3 * i + 1];
            *vertexPtr++ = attributes[j][3 * i + 2];
        }
    }
    auto vertexBuffer = new Qt3DCore::QBuffer(geometry);
    vertexBuffer->setData(vertexData);

    // Prepare vertex attributes
    for (uint32_t i = 0; i < attributeCount; i++) {
        auto vertexAttribute = new Qt3DCore::QAttribute(geometry);
        vertexAttribute->setName(*(attributeNames.begin() + i));
        vertexAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
        vertexAttribute->setVertexBaseType(Qt3DCore::QAttribute::VertexBaseType::Float);
        vertexAttribute->setBuffer(vertexBuffer);
        vertexAttribute->setVertexSize(3);
        vertexAttribute->setByteStride(stride * sizeof(float));
        vertexAttribute->setByteOffset(3 * i * sizeof(float));
        vertexAttribute->setCount(vertexCount);

        geometry->addAttribute(vertexAttribute);
    }

    delete[] vertices;
    delete[] normals;

    return geometry;
}