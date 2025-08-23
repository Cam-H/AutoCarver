//
// Created by cjhat on 2025-08-22.
//

#ifndef AUTOCARVER_BODY_H
#define AUTOCARVER_BODY_H

#include "fileIO/Serializable.h"
#include "geometry/Transformable.h"

#include "geometry/Mesh.h"


class Body : public Serializable, public Transformable {
public:

    Body();
    Body(const std::shared_ptr<Mesh>& mesh);

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;

    virtual void step(double delta);

    void setID(uint32_t ID);
    void setName(const std::string& name);

    void setMesh(const std::shared_ptr<Mesh>& mesh);

    void setLinearVelocity(glm::dvec3 velocity);
    void setAngularVelocity(glm::dvec3 velocity);

    [[nodiscard]] uint32_t getID() const;
    [[nodiscard]] const std::string& getName() const;

    const std::shared_ptr<Mesh>& mesh();

    [[nodiscard]] const glm::dvec3& getLinearVelocity() const;
    [[nodiscard]] const glm::dvec3& getAngularVelocity() const;

protected:

    uint32_t m_ID;
    std::string m_name;

    std::shared_ptr<Mesh> m_mesh;

    glm::dvec3 m_linearVelocity;
    glm::dvec3 m_angularVelocity;

};


#endif //AUTOCARVER_BODY_H
