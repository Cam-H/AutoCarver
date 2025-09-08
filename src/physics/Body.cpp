//
// Created by cjhat on 2025-08-22.
//

#include "Body.h"

Body::Body()
    : Body(nullptr)
{

}

Body::Body(const std::shared_ptr<Mesh>& mesh)
    : m_ID(0)
    , m_name()
    , m_mesh(mesh)
    , m_linearVelocity()
    , m_angularVelocity()
    , Transformable()
{

}

bool Body::serialize(std::ofstream& file) const
{
    if (!m_mesh->serialize(file)) return false;

    Serializer::writeTransform(file, m_transform);

    return true;
}

bool Body::deserialize(std::ifstream& file)
{
    m_mesh = std::make_shared<Mesh>(file);// TODO develop method to share meshes

    if (m_mesh != nullptr && m_mesh->vertexCount() > 0 && m_mesh->faceCount() > 0) {
        setTransform(Serializer::readTransform(file));

        return true;
    }

    return false;
}

void Body::step(double delta)
{
    globalTranslate(m_linearVelocity * delta);
    rotate(m_angularVelocity * delta);
}

void Body::setID(uint32_t ID)
{
    m_ID = ID;
}

void Body::setName(const std::string& name)
{
    m_name = name;
}

void Body::setMesh(const std::shared_ptr<Mesh>& mesh)
{
    m_mesh = mesh;
}

void Body::setLinearVelocity(glm::dvec3 velocity)
{
    m_linearVelocity = velocity;
}

void Body::setAngularVelocity(glm::dvec3 velocity)
{
    m_angularVelocity = velocity;
}

uint32_t Body::getID() const
{
    return m_ID;
}

const std::string& Body::getName() const
{
    return m_name;
}

std::shared_ptr<Mesh> Body::mesh()
{
    return m_mesh;
}

const glm::dvec3& Body::getLinearVelocity() const
{
    return m_linearVelocity;
}

const glm::dvec3& Body::getAngularVelocity() const
{
    return m_angularVelocity;
}