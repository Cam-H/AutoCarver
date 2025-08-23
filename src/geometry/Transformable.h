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
    void setRotation(const glm::dquat& quat);

    void translate(const glm::dvec3& translation);
    void rotate(const glm::dvec3& axis, double theta);
    void rotate(const glm::dvec3& w);

    void globalTranslate(const glm::dvec3& translation);
    void globalRotate(const glm::dvec3& axis, double theta);

    void scale(double scalar);

    void transform(const glm::dmat4x4& transform);

    void setTransform(glm::dmat4x4 transform);
    const glm::dmat4& getTransform();
    glm::dmat3 getRotation();

    virtual void moved();
    bool checkMoveState() const;
    void clearMoveState();

    [[nodiscard]] glm::dvec3 position() const;
    [[nodiscard]] glm::dvec3 up() const;

    inline static glm::dvec3 position(const glm::dmat4& transform)
    {
        return { transform[3][0], transform[3][1], transform[3][2] };
    }

    inline static glm::dmat3 rotation(const glm::dmat4& transform)
    {
        return {
                transform[0][0], transform[0][1], transform[0][2],
                transform[1][0], transform[1][1], transform[1][2],
                transform[2][0], transform[2][1], transform[2][2]
        };
    }

protected:

    glm::dmat4 m_transform;
    bool m_moved;
};


#endif //AUTOCARVER_TRANSFORMABLE_H
