//
// Created by cjhat on 2025-09-07.
//

#include "Sequence.h"

#include <gtc/quaternion.hpp>
#include <iostream>

Sequence::Sequence(const Profile* profile, const std::vector<glm::dvec3>& approach)
{
    assert(profile != nullptr);

    sets.emplace_back();
    for (const glm::dvec3& vertex : approach) {
        auto proj = profile->projected3D(glm::dvec2(vertex.x, vertex.y));
        sets[0].approach.emplace_back(proj, vertex.z);
    }

    normal = profile->normal();
}

Sequence::Sequence(const Profile* profile, const std::vector<SectionOperation::Set>& cuts)
{
    assert(profile != nullptr);

    std::cout << cuts.size() << " SEQUENCE!\n";
    for (const SectionOperation::Set& cut : cuts) {

        sets.emplace_back(
                profile->projected3D(cut.axis),
                profile->projected3D(cut.normal()),
                profile->projected3D(cut.travel)
        );

        for (const std::pair<glm::dvec2, double>& motion : cut.motions) {
            sets.back().motions.emplace_back(profile->projected3D(motion.first), motion.second);
        }

        for (const glm::dvec3& vertex : cut.approach) {
            std::cout << "TSI: " << vertex.x << " " << vertex.y << " " << vertex.z << " -> \n";

            auto proj = profile->projected3D(glm::dvec2(vertex.x, vertex.y));
//            std::cout << "TSF: " << proj.x << " " << proj.y << " " << proj.z << " -> \n";

            sets.back().approach.emplace_back(proj, vertex.z);
        }
    }

    normal = profile->normal();
}

void Sequence::transform(const glm::dquat& rotation, const glm::dvec3& translation)
{
    for (Set& set : sets) {
        set.axis = rotation * set.axis;
        set.normal = rotation * set.normal;
        set.travel = rotation * set.travel;

        for (std::pair<glm::dvec3, double>& motion : set.motions) {
            motion.first = rotation * motion.first + translation;
        }

        for (std::pair<glm::dvec3, double>& vertex : set.approach) {
            vertex.first = rotation * vertex.first + translation;
            std::cout << "TS: " << vertex.first.x << " " << vertex.first.y << " " << vertex.first.z << " " << vertex.second << "\n";
        }
    }

    normal = rotation * normal;
}

const glm::dvec3& Sequence::start() const
{
    assert(!sets.empty() && !sets[0].motions.empty());
    return sets[0].motions[0].first;
}

const glm::dvec3& Sequence::end() const
{
    assert(!sets.empty() && !sets.back().motions.empty());
    return sets.back().motions.back().first;
}

Axis3D Sequence::Set::axes(const glm::dvec3& forward) const
{
    Axis3D system(axis, normal);
    if (glm::dot(forward, system.zAxis) < 0) system.flipXZ();
    return system;
}


bool Sequence::Set::isApproach() const
{
    return type == 0;
}
bool Sequence::Set::isBlind() const
{
    return type == 1;
}
bool Sequence::Set::isMill() const
{
    return type == 2;
}