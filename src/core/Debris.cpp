//
// Created by cjhat on 2025-08-13.
//

#include "Debris.h"

#include "geometry/collision/Collision.h"



Debris::Debris()
    : CompositeBody()
    , m_inProcess(false)
{

}

// Deferred in case hulls are to be individually inserted
void Debris::initialize()
{
    remesh();

    m_hull = ConvexHull(m_mesh);

    prepareComponents();
    prepareTree();
}

void Debris::addFixedPlane(const Plane& plane)
{
    m_fixedPlanes.emplace_back(plane);
}

// System [Local] - xAxis = direction of cut
void Debris::queueCut(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& axis, double thickness, double theta)
{
    m_cuts.emplace_back(origin, normal, axis, thickness, theta);
}

void Debris::beginCut()
{
    if (m_cuts.empty() && !m_inProcess) return;

    CutOperation& cut = m_cuts[0];
    std::cout << "Cut: " << components().size() << " " << m_cuts.size() << " " << cut.axis.x << " " << cut.axis.y << " " << cut.axis.z
    << " | " << cut.normal.x << " " << cut.normal.y << " " << cut.normal.z << " | " << cut.theta << " " << "\n";

    double ht = 0.5 * cut.thickness + 1e-6;

    double hyt = ht, hxt = 0;
    if (std::abs(cut.theta) > 1e-6) {
        hyt *= sin(cut.theta);
        hxt = ht * cos(cut.theta);
    }

    auto filter = Plane(cut.origin - hxt * cut.axis, cut.axis);
    auto fwdPlane = Plane(cut.origin + hyt * cut.normal, cut.normal);
    auto revPlane = Plane(cut.origin - hyt * cut.normal, -cut.normal);

    std::vector<uint32_t> kerfs;

    // Split any hulls that pass through the cutting planes to limit effect to kerfs
    const uint32_t initialCount = components().size();
    for (uint32_t i = 0; i < initialCount; i++) {
        if (Collision::below(components()[i], filter)) continue; // Skip hulls behind the blade

        auto [min, max] = components()[i].extremes(cut.normal);
        double near = glm::dot(cut.normal, components()[i].vertices()[min] - cut.origin);
        double far = glm::dot(cut.normal, components()[i].vertices()[max] - cut.origin);
        std::cout << "H" << i << ": " << near << " " << far << " | " << ht << " " << hyt << " " << hxt << " ";

        bool nearPass = near < -hyt - 1e-6, farPass = far > hyt + 1e-6;

        uint32_t idx = i, count = components().size();

        std::cout << nearPass << " " << farPass << " ";

        // Try splitting against the near cut plane
        if (nearPass) {
            if (split(revPlane, idx)) idx = components().size() - 1;
        }

        // Try splitting against the far cut plane
        if (farPass) {
            if (split(fwdPlane, idx)) idx = components().size() - 1;
        }

        // If splits were made (Normal kerf) OR thin-hull case (Entire hull fits inside kerf)
        if (count < components().size() || !(nearPass || farPass)) {
            kerfs.emplace_back(idx);
        }

        std::cout << count << " " << components().size() << "\n";
    }

    std::cout << "KERFS: " << kerfs.size() << "\n";
    if (kerfs.empty()) throw std::runtime_error("[Debris] Failed to prepare cut");

    // Prepare sections
    for (uint32_t kerf : kerfs) {
        auto [idx, vertex] = components()[kerf].extreme(-cut.axis);

        cut.sections.emplace_back(
                kerf,
                glm::dot(cut.axis, vertex - cut.origin)
                );

        std::cout << "CS " << cut.sections.back().ts << " " << cut.sections.back().cutIndex << "\n";
    }

    updateConnections();
    m_inProcess = true;
}

void Debris::updateConnections()
{
    m_connections = std::vector<Connection>(components().size());

    // Identify anchored hulls
    for (uint32_t i = 0; i < m_connections.size(); i++) m_connections[i].anchor = isAnchor(components()[i]);

    // Identify hull contacts
    for (uint32_t i = 0; i < m_connections.size(); i++) {
        for (uint32_t j = i + 1; j < m_connections.size(); j++) {
            if (testContact(i, j)) {
                m_connections[i].contacts.emplace_back(j);
                m_connections[j].contacts.emplace_back(i);
            }
        }
    }

    // Count the number of anchors directly in contact to each hull
    for (Connection& con : m_connections) {
        for (uint32_t idx : con.contacts) {
            con.anchors += m_connections[idx].anchor;
        }
    }
}

void Debris::updateConnection(uint32_t index)
{
    // Revert anchors when they lose contact with the fixed planes
    if (m_connections[index].anchor && !isAnchor(components()[index])) {
        for (uint32_t link : m_connections[index].contacts) removeAnchor(index, link);
        m_connections[index].anchor = false;
        m_connections[index].tested = m_connections[index].anchors > 0;
    }

    // Remove contact links between newly separated hulls
    for (uint32_t i = 0; i < m_connections[index].contacts.size(); i++) {
        uint32_t link = m_connections[index].contacts[i];
        if (!testContact(index, link)) {

            removeLink(index, link);

            removeAnchor(link, index);
            m_connections[index].contacts.erase(m_connections[index].contacts.begin() + i);

            i--;
        }
    }
}

// Identify if the hull is directly in contact with the sculpture (Fixed planes used as a proxy)
bool Debris::isAnchor(const ConvexHull& hull) const
{
    for (const Plane& plane : m_fixedPlanes) {
        if (Collision::distance(hull, plane) < 1e-12) return true;
    }

    return false;
}

// Identify if the hull is indirectly in contact with an anchor
bool Debris::isAnchored(uint32_t idx) const
{
    return isAnchored(connections(idx));
}

// Identify if the hull is indirectly in contact with an anchor
bool Debris::isAnchored(const std::vector<uint32_t>& connections) const
{
    for (uint32_t idx : connections) {
        if (m_connections[idx].anchor || m_connections[idx].anchors > 0) return true;
    }

    return false;
}

bool Debris::testContact(uint32_t I0, uint32_t I1) const
{
    return Collision::distance(components()[I0], components()[I1]) < 1e-6;
}

std::vector<uint32_t> Debris::connections(uint32_t idx) const
{
    std::vector<uint32_t> connected = { idx };

    // Tracking visits to avoide backtracking
    auto visits = std::vector<bool>(m_connections.size(), false);
    visits[idx] = true;

    for (uint32_t i = 0; i < connected.size(); i++) {
        for (uint32_t j : m_connections[connected[i]].contacts) {
            if (!visits[j]) {
                connected.emplace_back(j);
                visits[j] = true;
            }
        }
    }

    return connected;
}


void Debris::completeCut()
{
    if (!m_cuts.empty()) m_cuts.pop_front();
    m_inProcess = false;
}

bool Debris::inProcess() const
{
    return m_inProcess;
}

// Shaves material from the next section (depth from the origin). Releases fragments as connections are removed
std::vector<std::shared_ptr<RigidBody>> Debris::removeMaterial(const glm::dvec3& normal, double depth)
{
    if (!m_cuts.empty()) {

        assert(m_connections.size() == components().size());

        Plane cutPlane(m_cuts[0].origin + m_cuts[0].axis * depth, normal); // m_cuts[0].axis

        // Remove material from all relevant kerfs and update connections
        for (uint32_t i = 0; i < m_cuts[0].sections.size(); i++) {
            if (m_cuts[0].sections[i].ts < depth) {
                auto remainder = Collision::fragment(components()[m_cuts[0].sections[i].cutIndex], cutPlane);
                if (remainder.isValid()) {
                    replace(remainder, m_cuts[0].sections[i].cutIndex);
                    updateConnection(m_cuts[0].sections[i].cutIndex);
                } else {
                    removeKerf(i);
                    i--;
                }
            }
        }

        auto fragments = tryFragmentRelease();

        remesh();

        return fragments;
    }

    return {};
}

void Debris::removeKerf(uint32_t index)
{
    index = m_cuts[0].sections[index].cutIndex;
    removeConnection(index);
    removeIndexed(index);
}

// Cleanly removes connection - Connected
void Debris::removeConnection(uint32_t index)
{
    uint32_t last = components().size() - 1;

    // Remove remaining connections to the element
    for (uint32_t idx : m_connections[index].contacts) removeLink(index, idx);

    // Correct indexing due to swap before removal
    if (index < last) {
        m_connections[index] = m_connections[last];

        for (uint32_t idx : m_connections[index].contacts){
            auto loc = std::find(m_connections[idx].contacts.begin(), m_connections[idx].contacts.end(), last);
            assert(loc != m_connections[idx].contacts.end());
            *loc = index;
        }
    }

    m_connections.pop_back();
}

void Debris::removeLink(uint32_t index, uint32_t link)
{
    auto loc = std::find(m_connections[link].contacts.begin(), m_connections[link].contacts.end(), index);
    assert(loc != m_connections[link].contacts.end());
    m_connections[link].contacts.erase(loc);
    removeAnchor(index, link);
}

// Remove the connection [anchor] from link. Marks for testing if it is not directly anchored
void Debris::removeAnchor(uint32_t anchor, uint32_t link)
{
    if (m_connections[anchor].anchor) m_connections[link].anchors--;
    m_connections[link].tested = m_connections[link].anchors > 0;
}

// Update section (cut) indices if it is not the last hull before removing the hull from the body
void Debris::removeIndexed(uint32_t index)
{
    uint32_t last = components().size() - 1;

    // Remove kerf associated with the hull (If it exists)
    for (uint32_t i = 0; i < m_cuts[0].sections.size(); i++) {
        if (m_cuts[0].sections[i].cutIndex == index) {
            m_cuts[0].sections.erase(m_cuts[0].sections.begin() + i);
            break;
        }
    }

    // CompositeBody swaps index & last hull prior to removal - Adjust index given that fact
    if (index < last) {
        for (CutOperation& cut : m_cuts) {
            for (Kerf& sec : cut.sections) {
                if (sec.cutIndex == last) sec.cutIndex = index;
            }
        }
    }

    remove(index);
}

std::vector<std::shared_ptr<RigidBody>> Debris::tryFragmentRelease()
{
    std::vector<std::shared_ptr<RigidBody>> fragments;

    for (uint32_t i = 0; i < m_connections.size(); i++) {
        if (!m_connections[i].tested) { // Only check hulls that might be released (Recent total loss of direct contact with anchors)
            m_connections[i].tested = true;

            auto connected = connections(i);

            if (!isAnchored(connected)) {

                // Sort in descending order so that hull removal is index-consistent
                std::sort(connected.begin(), connected.end(), [](uint32_t a, uint32_t b) {
                    return a > b;
                });

                fragments.emplace_back(prepareFragment(connected));

                // Remove released hulls
                for (uint32_t idx : connected) {
                    removeConnection(idx);
                    removeIndexed(idx);
                }
            } else { // Block retests for connected hulls (In case they were also triggered)
                for (uint32_t idx : connected) m_connections[idx].tested = true;
            }
        }
    }

    return fragments;
}

// Prepare fragment, attaching all connected hulls together for each
std::shared_ptr<RigidBody> Debris::prepareFragment(const std::vector<uint32_t>& connected)
{
    std::shared_ptr<RigidBody> fragment = nullptr;
    if (connected.size() == 1) fragment = std::make_shared<RigidBody>(components()[connected[0]]);
    else {
        std::vector<ConvexHull> set;
        set.reserve(connected.size());

        for (uint32_t idx : connected) if (components()[idx].isValid()) set.push_back(components()[idx]);
        fragment = std::make_shared<CompositeBody>(set);
    }

    fragment->setTransform(getTransform());
    fragment->setType(RigidBody::Type::DYNAMIC);

    return fragment;
}

void Debris::print() const
{
    std::cout << "[Debris] hulls: " << components().size() << " (" << m_connections.size() << "), cuts: " << m_cuts.size() << ", sections: " << m_cuts[0].sections.size() << "\n";

    for (uint32_t i = 0; i < m_connections.size(); i++) {
        std::cout << "CON" << i << ": ( anchors: " << m_connections[i].anchor << ", " << m_connections[i].anchors
                  << ", anchored: " << (m_connections[i].anchor || isAnchored(i))
                  << ", tested: " << m_connections[i].tested  << "): ";

        for (uint32_t j : m_connections[i].contacts) std::cout << j << " ";
        std::cout << "\n";
    }
}