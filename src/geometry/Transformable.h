//
// Created by Cam on 2025-04-17.
//

#ifndef AUTOCARVER_TRANSFORMABLE_H
#define AUTOCARVER_TRANSFORMABLE_H

#include <glm/glm.hpp>

class Transformable {
public:

    Transformable();

    void setPosition(const glm::vec3& position);
    void setRotation(const glm::vec3& euler);

    void translate(const glm::vec3& translation);
    void rotate(const glm::vec3& axis, float theta);
    void rotate(const glm::vec3& w);

    void globalTranslate(const glm::vec3& translation);
    void globalRotate(const glm::vec3& axis, float theta);

    void transform(const glm::mat4x4& transform);

    void setTransform(glm::mat4x4 transform);
    const glm::mat4x4& getTransform();

    virtual void moved();

    [[nodiscard]] glm::vec3 position() const;
    [[nodiscard]] glm::vec3 up() const;

protected:

    glm::mat4 m_transform;
};


#endif //AUTOCARVER_TRANSFORMABLE_H
