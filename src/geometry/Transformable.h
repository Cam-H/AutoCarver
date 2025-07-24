//
// Created by Cam on 2025-04-17.
//

#ifndef AUTOCARVER_TRANSFORMABLE_H
#define AUTOCARVER_TRANSFORMABLE_H

#include <glm.hpp>

class Transformable {
public:

    Transformable();

    void setPosition(const glm::dvec3& position);
    void setRotation(const glm::dvec3& euler);

    void translate(const glm::dvec3& translation);
    void rotate(const glm::dvec3& axis, double theta);
    void rotate(const glm::dvec3& w);

    void globalTranslate(const glm::dvec3& translation);
    void globalRotate(const glm::dvec3& axis, double theta);

    void transform(const glm::dmat4x4& transform);

    void setTransform(glm::dmat4x4 transform);
    const glm::dmat4& getTransform();
    glm::dmat3 getRotation();

    virtual void moved();

    [[nodiscard]] glm::dvec3 position() const;
    [[nodiscard]] glm::dvec3 up() const;

protected:

    glm::dmat4 m_transform;
};


#endif //AUTOCARVER_TRANSFORMABLE_H
