//
// Created by Cam on 2024-11-10.
//

#include "Sculpture.h"

#include "geometry/MeshBuilder.h"
#include "geometry/Collision.h"


Sculpture::Sculpture(const std::shared_ptr<Mesh>& model, float width, float height)
    : CompositeBody(MeshBuilder::box())
    , m_width(width)
    , m_height(height)
    , m_rotation(0)
    , m_scalar(1.0f)
    , m_preserveDebris(false)
    , m_step(0)
    , m_formStep(0)
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
//    model->translate({ 0, -m_height / 2, 0 });
}

void Sculpture::prepareBox()
{
    m_mesh->setBaseColor(baseColor());
    m_mesh->scale({ m_width, m_height, m_width });
    m_mesh->translate({ 0, m_height / 2, 0 });
    m_hull = ConvexHull(m_mesh);
    prepareTree();
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
    m_hull = modelBody->hull();
    CompositeBody::restore();
    m_mesh = std::make_shared<Mesh>(m_hull);
    m_mesh->setFaceColor(baseColor());
}

void Sculpture::moved()
{
    modelBody->setTransform(m_transform);
}

void Sculpture::queueSection(const glm::vec3& origin, const glm::vec3& normal)
{
    m_operations.emplace_back(origin, normal);
}

void Sculpture::queueSection(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& normal, bool external)
{
    m_operations.emplace_back();

    auto &operation = m_operations[m_operations.size() - 1];
    operation.surfaces.reserve(2);
    operation.limits.reserve(1);

    operation.surfaces.emplace_back(a, glm::normalize(glm::cross(normal, b - a)));
    operation.surfaces.emplace_back(b, glm::normalize(glm::cross(normal, c - b)));

    if (!external) {
        operation.limits.emplace_back(c, glm::normalize(glm::cross(normal, a - c)));
//        operation.limits.emplace_back(b, -operation.limits[0].second);

    }
//    operation.limits.emplace_back(a, glm::normalize(b - a));
//    operation.limits.emplace_back(c, glm::normalize(b - c));

}

//void Sculpture::queueSection(const std::vector<glm::vec3>& border, const glm::vec3& normal)
//{
//    if (border.size() < 3) return;
//    else if (border.size() == 3) queueSection(border[0], border[1], border[2], normal);
//    else {
//        throw std::runtime_error("[Sculpture] Sections more complicated than triangles have yet to be developed!");
//    }
//}

bool Sculpture::applySection()
{
    if (m_step < m_operations.size()) {
        uint32_t step = m_step++;

        switch (m_operations[step].surfaces.size()) {
            case 1:
                planarSection(m_operations[step].surfaces[0]);
                break;
            case 2:
                if (hulls().empty()) CompositeBody::restore(); // TODO move when extending to > 2 surface cuts
                triangleSection(m_operations[step].surfaces[0], m_operations[step].surfaces[1], m_operations[step].limits);
                break;
            default:
                std::cout << "[Sculpture] Queued section not handled. Not currently supported\n";
        }
    }

    return false;
}

bool Sculpture::planarSection(const Plane& plane)
{
    if (!Collision::test(m_hull, plane)) return false;

    // TODO expand for multiple hulls
    if (m_preserveDebris) {
        auto fragments = Collision::fragments(m_hull, plane);
        m_hull = fragments.first;

        if (!fragments.second.empty()) prepareFragment(fragments.second);
    } else {
        m_hull = Collision::fragment(m_hull, plane);
    }

    remesh();

    // Highlight cut faces
    m_mesh->setFaceColor(m_mesh->matchFace(plane.normal), m_highlightColor);

    return true;
}

bool Sculpture::inLimit(const ConvexHull& hull, const Plane& limit)
{
    return !Collision::below(hull, limit);
}

bool Sculpture::inLimit(const ConvexHull& hull, const std::vector<Plane>& limits)
{
    for (const auto& limit : limits) {
        if (!inLimit(hull, limit)) return false;
    }

    return true;
}

bool Sculpture::triangleSection(const Plane& planeA, const Plane& planeB, const std::vector<Plane>& limits)
{
    const auto& fragments = hulls();
    for (uint32_t i = 0; i < fragments.size(); i++) {

        auto aFragments = Collision::fragments(fragments[i], { planeA.origin, -planeA.normal });
        auto bFragments = Collision::fragments(aFragments.second, { planeB.origin, -planeB.normal });
        if (bFragments.second.empty()) continue;

//        std::cout << i << " " << limits.size() << " | "
//        << inLimit(bFragments.second, limits)
//        << " A: " << !aFragments.first.empty()
//        << " B: " << !bFragments.first.empty() << "\n";

        if (inLimit(bFragments.second, limits)) {
            if (!aFragments.first.empty()) { // Intersection with plane A
                replace(aFragments.first, i);

                if (!bFragments.first.empty()) add(bFragments.first); // Intersection with plane B

            } else { // Entirely beyond plane A

                if (!bFragments.first.empty()) replace(bFragments.first, i); // Intersection with plane B
                else { // Hull entirely within the cut - to be deleted
                    remove(i);
                    i--;
                }
            }
        }
    }

//    std::cout << "R " << m_hulls.size() << "\n";

    // Manually remesh only if no hulls were merged (If they are the superclass already remeshes)
    if (!tryMerge()) remesh();
//    remesh();

    return true;
}

bool Sculpture::form()
{
    std::cout << "F Step: " << m_formStep + 1 << " / " << modelBody->mesh()->faceCount() << "\n";
//    if (m_formStep < modelBody->mesh()->faceCount() && !m_hulls.empty()) {
//
//        auto normal = modelBody->mesh()->faces().normal(m_formStep);
//        auto border = modelBody->mesh()->faces().faceBorder(m_formStep, modelBody->mesh()->vertices().vertices());
//        Sphere bounds = Sphere::enclose(border);
//
//        std::cout << border.size() << " " << bounds.center.x << " " << bounds.center.y << " " << bounds.center.z << " " << bounds.radius << " B\n";
//        std::cout << "Norm: " << normal.x << " " << normal.y << " " << normal.z << "\n";
//        for (uint32_t i = 0; i < m_hulls.size(); i++) {
////            if (m_hulls[i].intersects(bounds)) {
////                auto fragments = m_hulls[i].fragments(border[0], -normal);
////                m_hulls[i] = fragments.first;
////
////                std::cout << "Int: " << i << " " << fragments.first.vertexCount() << " " << fragments.second.vertexCount() << "\n";
////            }
//
//        }
//
//        std::cout << "Border: " << border.size() << "\n";
//
//        m_formStep++;
//        remesh();
//    }

    return false;
}

void Sculpture::remesh()
{
    CompositeBody::remesh();
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