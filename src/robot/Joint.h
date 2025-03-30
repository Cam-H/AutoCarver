//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_JOINT_H
#define AUTOCARVER_JOINT_H

#include <glm/glm.hpp>

struct DHParameter {
    float len;
    float dist;
    float alpha;
    float theta;
};

class Joint {
public:

    enum class Type{
        NONE = 0, PRISMATIC, REVOLUTE
    };

    Joint(Joint::Type type, const DHParameter& parameters, float initialValue = 0.0f);

    void recalculate();

    void setJointLimits(float lower, float upper);
    void setValue(float value);

    [[nodiscard]] const DHParameter& getParameters() const;

    [[nodiscard]] float getLowerLimit() const;
    [[nodiscard]] float getUpperLimit() const;
    [[nodiscard]] float getValue() const;
    bool withinLimits(float value) const;
//    const glm::vec3& getCenter() const;

//    [[nodiscard]] const glm::mat4x4& getTransform() const;
    [[nodiscard]] glm::mat4x4 calculateHTM() const;
    [[nodiscard]] glm::mat4x4 calculateHTM(float value) const;

    [[nodiscard]] glm::mat3x3 calculateHRM() const;
    [[nodiscard]] glm::mat3x3 calculateHRM(float value) const;

    [[nodiscard]] const glm::mat4x4& getHTM() const;

    [[nodiscard]] glm::mat4x4 localRotationMatrix() const;


protected:

    [[nodiscard]] float distance(float value) const;
    [[nodiscard]] float angle(float value) const;

protected:

    Joint::Type m_jointType;
    DHParameter m_parameters;

    float m_lowerLimit;
    float m_upperLimit;

    float m_value;

private:

    glm::mat4x4 m_htm;

};


#endif //AUTOCARVER_JOINT_H
