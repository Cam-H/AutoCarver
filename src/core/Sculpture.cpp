//
// Created by Cam on 2024-11-10.
//

#include "Sculpture.h"

#include "geometry/MeshBuilder.h"

Sculpture::Sculpture(const std::shared_ptr<Mesh>& model, float width, float height)
    : CompositeBody(MeshBuilder::box())
    , m_width(width)
    , m_height(height)
    , m_rotation(0)
    , m_scalar(1.0f)
    , m_preserveDebris(false)
    , m_step(0)
    , m_baseColor(0.7f, 0.7f, 0.7f)
    , m_highlightColor(0.3f, 0.3f, 0.8f)
{

    scaleToFit(model, m_width, m_height);
    prepareBox();

    modelBody = std::make_shared<RigidBody>(model);

    std::cout << "Volume ratio: " << 100 * bulkUsageRatio() << "% material usage, " << 100 * remainderRatio() << "% Remaining\n";
}

void Sculpture::scaleToFit(const std::shared_ptr<Mesh>& model, float width, float maxHeight)
{
    // Find the maximum dimensions of the mesh
    float xNear, xFar, yNear, yFar, zNear, zFar;
    model->xExtents(xNear, xFar);
    model->yExtents(yNear, yFar);
    model->zExtents(zNear, zFar);

    model->translate({ -(xNear + xFar) / 2, -yNear, -(zNear + zFar) / 2 }); // Center the model in span

    m_rotation = atanf((xFar - xNear) / (zFar - zNear));
    model->rotate({ 0, 1, 0 }, m_rotation); // Rotate the model to make the best use of diagonal space

    // Re-evaluate extents to figure out scaling requirements
    model->xExtents(xNear, xFar);
    model->zExtents(zNear, zFar);

    model->translate({ -(xNear + xFar) / 2, 0, -(zNear + zFar) / 2 }); // Adjust model in the center

    m_height = yFar - yNear;

    // Calculate the scale factor to make the mesh as large as possible, within the specified limits
    m_scalar = std::min(width / (xFar - xNear), width / (zFar - zNear));
    if (m_height * m_scalar > maxHeight) { // Adjust scale to ensure the model does not exceed the maximum allowable height
        m_scalar = maxHeight / m_height;
    }

    m_height *= m_scalar;

    model->scale(m_scalar); // Scale model to fit within specified limits
}

void Sculpture::prepareBox()
{
    m_mesh->setBaseColor(m_baseColor);
    m_mesh->scale({ m_width, m_height, m_width });
    m_mesh->translate({ 0, m_height / 2, 0 });
    m_hull = ConvexHull(m_mesh);
}

void Sculpture::prepareFragment(const ConvexHull& hull)
{
    m_fragments.emplace_back(hull);
    m_fragments[m_fragments.size() - 1].setType(RigidBody::Type::DYNAMIC);
}

void Sculpture::update()
{

}

void Sculpture::restore()
{
    m_mesh = MeshBuilder::box();
    prepareBox();
}
void Sculpture::restoreAsHull()
{
    m_mesh = std::make_shared<Mesh>(modelBody->hull());
    m_mesh->setFaceColor(m_baseColor);
}

void Sculpture::moved()
{
    modelBody->setTransform(m_transform);
}

void Sculpture::recordSection(const glm::vec3& origin, const glm::vec3& normal)
{
    m_operations.emplace_back(origin, normal);
}
void Sculpture::recordSection(const std::vector<glm::vec3>& border, const glm::vec3& normal)
{
    if (border.size() < 3) return;

    m_operations.emplace_back();

    auto &surfaces = m_operations[m_operations.size() - 1].surfaces;
    surfaces.reserve(border.size() - 1);

    for (uint32_t i = 0; i < border.size() - 1; i++) {
        surfaces.emplace_back(border[i], glm::normalize(glm::cross(normal, border[i + 1] - border[i])));
    }
}

void Sculpture::section()
{
    if (m_step < m_operations.size()) {
        if (m_operations[m_step].surfaces.size() == 1)
            section(m_operations[m_step].surfaces[0].first, m_operations[m_step].surfaces[0].second);
        else section(m_operations[m_step].surfaces);

        m_step++;
    }
}

void Sculpture::section(const glm::vec3& origin, const glm::vec3& normal)
{
    if (m_preserveDebris) {
        auto fragments = m_hull.fragments(origin, normal);
        m_hull = fragments.first;

        if (!fragments.second.empty()) prepareFragment(fragments.second);
    } else {
        m_hull = m_hull.fragment(origin, normal);
    }

    remesh();

    // Highlight cut faces
    m_mesh->setFaceColor(m_mesh->matchFace(normal), m_highlightColor);
}

void Sculpture::section(const std::vector<std::pair<glm::vec3, glm::vec3>>& surfaces)
{
    if (m_hulls.empty()) m_hulls.push_back(m_hull);

//    auto fragments = m_hull.fragments(origin, normal);
//    m_hull = fragments.first;
    std::cout << "section " << surfaces.size() << "\n";

    if (surfaces.size() == 2) triangleSection(surfaces[0], surfaces[1]);
    else std::cout << "Greater sections not handled [Sculpture]\n";

    remesh();

}

void Sculpture::triangleSection(const std::pair<glm::vec3, glm::vec3>& planeA, const std::pair<glm::vec3, glm::vec3>& planeB)
{

    auto normal = glm::normalize(glm::cross(planeA.second, planeB.second));
    auto pA = -glm::normalize(glm::cross(normal, planeA.second));
    auto pB = glm::normalize(glm::cross(normal, planeB.second));
    auto border = { planeB.first + 2.0f * pA, planeB.first, planeB.first + 2.0f * pB };
    auto mesh = MeshBuilder::extrude(border, normal, 5.0f);
    mesh->translate(-normal * 2.5f);
    m_hulls.emplace_back(mesh);

    return;
    uint32_t count = m_hulls.size();

    for (uint32_t i = 0; i < count; i++) {
        auto fragments = m_hulls[i].fragments(planeA.first, planeA.second);
        uint8_t status = !fragments.first.empty() + 2 * !fragments.second.empty();

        std::cout << i << " " << count << " " << m_hulls.size() << " " << (uint32_t)status << "~~\n";
        switch (status) {
            case 1: // The hull is not entirely below the cutting plane
                break;
            case 2: // The hull is entirely above the cutting plane
                std::swap(m_hulls[i], m_hulls[count - 1]);
                count--;
                i--;
                break;
            case 3: // The hull is bisected by the cutting plane
                m_hulls[i] = fragments.first;
                m_hulls.push_back(fragments.second);
                break;
            default: // 0 - No hull on either side of cutting plane (Impossible unless the source is empty)
                std::cout << "[Sculpture] Error! Empty hulls are present\n";
        }
    }

    // Operate over fragments above the first cutting plane
    for (uint32_t i = count; i < m_hulls.size(); i++) {
        auto fragments = m_hulls[i].fragments(planeB.first, planeB.second);

        std::cout << fragments.first.empty() << " " << fragments.second.empty() << " " << i << " " << count << "||\n";
        if (fragments.first.empty()) {
            m_hulls.erase(m_hulls.begin() + i);
            i--;
        } else m_hulls[i] = fragments.first;

        if (!fragments.second.empty()) m_fragments.emplace_back(fragments.second);
    }
}

void Sculpture::remesh()
{
    CompositeBody::remesh();

    // Prepare styling for the fragments
    m_mesh->setFaceColor(m_baseColor);
}

const std::shared_ptr<Mesh>& Sculpture::sculpture()
{
    return m_mesh;
}

const std::shared_ptr<RigidBody>& Sculpture::model()
{
    return modelBody;
}


float Sculpture::width() const
{
    return m_width;
}
float Sculpture::height() const
{
    return m_height;
}
float Sculpture::rotation() const
{
    return m_rotation;
}

float Sculpture::scalar() const
{
    return m_scalar;
}

float Sculpture::initialVolume() const
{
    return m_width * m_width * m_height;
}
float Sculpture::currentVolume() const
{
    return m_mesh->volume();
}
float Sculpture::finalVolume() const
{
    return modelBody->volume();
}

float Sculpture::bulkUsageRatio() const
{
    return finalVolume() / initialVolume();
}
float Sculpture::remainderRatio() const
{
    float fVolume = finalVolume();

    return (currentVolume() - fVolume) / (initialVolume() - fVolume);
}