//
// Created by cjhat on 2025-08-21.
//

#include "RenderBuffer.h"

#include "core/Scene.h"
#include "physics/RigidBody.h"

RenderBuffer::RenderBuffer()
    : m_latest(0)
    , m_active(1)
{

}

void RenderBuffer::update(const Scene* scene)
{
    uint32_t idx = nextToUpdate();
    m_buffer[idx] = std::vector<Item>();
    m_buffer[idx].reserve(scene->bodyCount());

    for (const std::shared_ptr<RigidBody>& body : scene->bodies()) {
        m_buffer[idx].push_back(Item{
            body->getID(),
            body->getTransform(),
            body->mesh(),
            body->hullMesh(),
            body->boundingSphere()
        });
    }

    m_mutex.lock();

    m_latest = idx;

    m_mutex.unlock();
}

const std::vector<RenderBuffer::Item>& RenderBuffer::latest()
{
    m_mutex.lock();
    m_active = m_latest;
    m_mutex.unlock();

    return m_buffer[m_active];
}

uint32_t RenderBuffer::nextToUpdate()
{
    m_mutex.lock();
    uint32_t idx = 0;
    if (idx == m_active || idx == m_latest) idx++;
    if (idx == m_active || idx == m_latest) idx++;
    m_mutex.unlock();

    return idx;
}