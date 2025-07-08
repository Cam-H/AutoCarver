//
// Created by Cam on 2025-07-06.
//

#include "CompositeBody.h"

#include "geometry/MeshBuilder.h"

CompositeBody::CompositeBody(const std::shared_ptr<Mesh>& mesh)
    : RigidBody(mesh)
{

}

CompositeBody::CompositeBody(const std::vector<ConvexHull>& hulls)
    : RigidBody(MeshBuilder::composite(hulls))
    , m_hulls(hulls)
{

}

void CompositeBody::remesh()
{
    m_mesh = MeshBuilder::composite(m_hulls);
}