//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_JOINT_H
#define AUTOCARVER_JOINT_H

#include <glm.hpp>

struct DHParameter {
    double len;
    double dist;
    double alpha;
    double theta;
};

class Joint {
public:

    enum class Type{
        NONE = 0, PRISMATIC, REVOLUTE
    };

    Joint(Joint::Type type, const DHParameter& parameters, double initialValue = 0.0f);

    void recalculate();

    void setJointLimits(double lower, double upper);
    void setValue(double value);

    [[nodiscard]] const DHParameter& getParameters() const;

    [[nodiscard]] double getLowerLimit() const;
    [[nodiscard]] double getUpperLimit() const;
    [[nodiscard]] double getRange() const;

    [[nodiscard]] double getValue() const;
    [[nodiscard]] bool withinLimits(double value) const;
    [[nodiscard]] double remainingLimit(double value) const;
//    const glm::dvec3& getCenter() const;

//    [[nodiscard]] const glm::dmat4x4& getTransform() const;
    [[nodiscard]] glm::dmat4 calculateHTM() const;
    [[nodiscard]] glm::dmat4 calculateHTM(double value) const;

    [[nodiscard]] glm::dmat3 calculateHRM() const;
    [[nodiscard]] glm::dmat3 calculateHRM(double value) const;

    [[nodiscard]] const glm::dmat4& getHTM() const;

    [[nodiscard]] glm::dmat4x4 localRotationMatrix() const;


protected:

    [[nodiscard]] double distance(double value) const;
    [[nodiscard]] double angle(double value) const;

protected:

    Joint::Type m_jointType;
    DHParameter m_parameters;

    double m_lowerLimit;
    double m_upperLimit;

    double m_value;

private:

    glm::dmat4x4 m_htm;

};


#endif //AUTOCARVER_JOINT_H
