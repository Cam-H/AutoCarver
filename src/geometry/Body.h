//
// Created by Cam on 2024-09-20.
//

#ifndef AUTOCARVER_BODY_H
#define AUTOCARVER_BODY_H

#include <fstream>

// Mesh manipulation

#include <vector>
#include <glm/glm.hpp>

#include "fileIO/Serializable.h"

#include "Mesh.h"
#include "ConvexHull.h"
#include "Sphere.h"

#include "Transformable.h"


class Body : public Serializable, public Transformable {
public:

    explicit Body(const std::string& filename);
    explicit Body(const std::shared_ptr<Mesh>& mesh);

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

    void setMesh(const std::shared_ptr<Mesh>& mesh, bool doColliderUpdate = false);

    void setLayer(uint32_t layer);
    void setMask(uint32_t mask);

    void updateColliders();
    void prepareColliderVisuals();

    bool isManifold();
    float area();
    float volume();

    const std::shared_ptr<Mesh>& mesh();

    const ConvexHull &hull() const;
    const Sphere& boundingSphere() const;

    const std::shared_ptr<Mesh>& hullMesh();
    const std::shared_ptr<Mesh>& bSphereMesh();


    [[nodiscard]] uint32_t layer() const;
    [[nodiscard]] uint32_t mask() const;
    [[nodiscard]] bool scan(const std::shared_ptr<Body>& body) const;

    bool boundaryCollision(const std::shared_ptr<Body>& body);

    bool collides(const std::shared_ptr<Body>& body);
    bool collision(const std::shared_ptr<Body>& body, glm::vec3& offset);

    EPA collision(const std::shared_ptr<Body>& body);

private:

    void prepareHullVisual();
    void prepareSphereVisual();


    void cacheCollision(const std::shared_ptr<Body>& body, const std::pair<uint32_t, uint32_t>& start);
    std::pair<uint32_t, uint32_t> cachedCollision(const std::shared_ptr<Body>& body);

    void evaluateManifold();
    void calculateArea();
    void calculateVolume();

protected:

    std::shared_ptr<Mesh> m_mesh;

    ConvexHull m_hull;
    bool m_hullOK;

    Sphere m_boundingSphere;

    uint32_t m_layer;
    uint32_t m_mask;

    std::shared_ptr<Mesh> m_hullMesh;
    std::shared_ptr<Mesh> m_sphereMesh;
    bool m_colliderVisualsEnable;


    bool m_isManifold;
    float m_area;
    float m_volume;

    bool m_isManifoldOK;
    bool m_areaOK;
    bool m_volumeOK;

private:



};


#endif //AUTOCARVER_BODY_H
