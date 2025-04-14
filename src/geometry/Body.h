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


class Body : public Serializable {
public:

    explicit Body(const std::shared_ptr<Mesh> &mesh);

    explicit Body(const std::string& filename);

    ~Body();

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

    void setMesh(const std::shared_ptr<Mesh>& mesh, bool recalculateHull = false);

    void setPosition(const glm::vec3& position);

    void translate(const glm::vec3& translation);
    void rotate(const glm::vec3& axis, float theta);

    void globalTranslate(const glm::vec3& translation);
    void globalRotate(const glm::vec3& axis, float theta);

    void transform(const glm::mat4x4& transform);

    void setTransform(glm::mat4x4 transform);
    const glm::mat4x4& getTransform();

    void updateHull();
    void prepareHullMesh();

    bool isManifold();
    float area();
    float volume();

    const std::shared_ptr<Mesh>& mesh();

    const ConvexHull &hull();
    const ConvexHull &hull() const;

    const std::shared_ptr<Mesh>& hullMesh();

    bool collides(const std::shared_ptr<Body>& body);
    bool collision(const std::shared_ptr<Body>& body, glm::vec3& offset);

    EPA collision(const std::shared_ptr<Body>& body);

private:

    void cacheCollision(const std::shared_ptr<Body>& body, const std::pair<uint32_t, uint32_t>& start);
    std::pair<uint32_t, uint32_t> cachedCollision(const std::shared_ptr<Body>& body);

    void evaluateManifold();
    void calculateArea();
    void calculateVolume();

protected:

    std::shared_ptr<Mesh> m_mesh;

    ConvexHull m_hull;
    std::shared_ptr<Mesh> m_hullMesh;

    bool m_hullOK;

    glm::mat4x4 m_transform;


    bool m_isManifold;
    float m_area;
    float m_volume;

    bool m_isManifoldOK;
    bool m_areaOK;
    bool m_volumeOK;

private:



};


#endif //AUTOCARVER_BODY_H
