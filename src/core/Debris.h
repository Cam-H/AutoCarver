//
// Created by cjhat on 2025-08-13.
//

#ifndef AUTOCARVER_DEBRIS_H
#define AUTOCARVER_DEBRIS_H

#include <deque>

#include "physics/CompositeBody.h"

#include "geometry/primitives/Plane.h"
#include "robot/Pose.h"

// Provides means to track the progress of cuts enacted during the sculpting process, simulating changing geometry & releasing fragments as required
class Debris : public CompositeBody {
public:

    explicit Debris();

    void initialize();

    void addFixedPlane(const Plane& plane);

    void queueCut(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& axis, double thickness, double theta);
    void beginCut();
    void completeCut();

    [[nodiscard]] bool inProcess() const;

    std::vector<std::shared_ptr<RigidBody>> removeMaterial(const glm::dvec3& normal, double depth);

    void print() const;

private:

    struct Connection {
        Connection() : anchors(0), anchor(false), tested(false) {}

        std::vector<uint32_t> contacts;
        uint32_t anchors; // Number of anchors in direct contact with the hull
        bool anchor; // Whether the hull itself is an anchor
        bool tested; // Whether connectivity should be tested after a recent change
    };

    struct Kerf {
        Kerf(uint32_t cut, double ts) : cutIndex(cut), ts(ts) {}

        uint32_t cutIndex;

        double ts; // Min distance from origin
    };

    struct CutOperation {
        CutOperation(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& axis, double thickness, double theta) : origin(origin), normal(normal), axis(axis), thickness(thickness), theta(theta) {}

        glm::dvec3 origin; // Center position between limit planes, from where axis begins
        glm::dvec3 normal; // Normal of limit plane

        glm::dvec3 axis;
        double thickness;
        double theta; // Angle between blade and normal

        std::vector<Kerf> sections;
    };

    void updateConnections();
    void updateConnection(uint32_t index);

    void removeKerf(uint32_t index);

    void removeConnection(uint32_t index);
    void removeLink(uint32_t index, uint32_t link);
    void removeAnchor(uint32_t anchor, uint32_t link);

    void removeIndexed(uint32_t index);

    [[nodiscard]] bool isAnchor(const ConvexHull& hull) const;
    [[nodiscard]] bool isAnchored(uint32_t idx) const;
    [[nodiscard]] bool isAnchored(const std::vector<uint32_t>& connections) const;

    [[nodiscard]] bool testContact(uint32_t I0, uint32_t I1) const;

    [[nodiscard]] std::vector<uint32_t> connections(uint32_t idx) const;

    std::vector<std::shared_ptr<RigidBody>> tryFragmentRelease();
    std::shared_ptr<RigidBody> prepareFragment(const std::vector<uint32_t>& connected);


private:

    std::vector<Plane> m_fixedPlanes; // Retains any hulls in contact (even indirect) with any plane
    std::vector<Connection> m_connections; // List of hulls and their contacts

    std::deque<CutOperation> m_cuts; // Record of remaining operations to be conducted
    bool m_inProcess;

};


#endif //AUTOCARVER_DEBRIS_H
