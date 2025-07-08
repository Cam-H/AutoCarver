//
// Created by Cam on 2025-07-06.
//

#ifndef AUTOCARVER_COMPOSITEBODY_H
#define AUTOCARVER_COMPOSITEBODY_H

#include "RigidBody.h"

class Mesh;

class CompositeBody : public RigidBody {
public:

    explicit CompositeBody(const std::shared_ptr<Mesh>& mesh);
    explicit CompositeBody(const std::vector<ConvexHull>& hulls);

    virtual void remesh();

protected:

    std::vector<ConvexHull> m_hulls;
};


#endif //AUTOCARVER_COMPOSITEBODY_H
