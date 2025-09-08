//
// Created by cjhat on 2025-09-07.
//

#include "Sequence.h"

#include <gtc/quaternion.hpp>

Sequence::Sequence(const Profile* profile, const std::vector<SectionOperation::Set>& cuts)
{
    assert(profile != nullptr);

    for (const SectionOperation::Set& cut : cuts) {

        sets.emplace_back(
                profile->projected3D(cut.axis),
                profile->projected3D(cut.normal),
                profile->projected3D(cut.travel)
        );

        for (const std::pair<glm::dvec2, double>& motion : cut.motions) {
            sets.back().motions.emplace_back(profile->projected3D(motion.first), motion.second);
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

        for (std::pair<glm::dvec3, double>& motion : set.motions) motion.first = rotation * motion.first + translation;
    }

    normal = rotation * normal;
}

const glm::dvec3& Sequence::start() const
{
    assert(!sets.empty() && !sets[0].motions.empty());
    return sets[0].motions[0].first;
}

Pose Sequence::startPose(const glm::dvec3& forward) const
{
    assert(!sets.empty() && !sets[0].motions.empty());
    return Pose(sets[0].motions[0].first, sets[0].axes(forward));
}

Axis3D Sequence::Set::axes(const glm::dvec3& forward) const
{
    return Axis3D::faceAligned(normal, forward, true);
}