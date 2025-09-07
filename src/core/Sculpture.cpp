//
// Created by Cam on 2024-11-10.
//

#include "Sculpture.h"

#include "geometry/MeshBuilder.h"
#include "geometry/collision/Collision.h"

#include "core/Debris.h"

Sculpture::Sculpture()
    : CompositeBody()
    , m_width(1.0)
    , m_height(1.0)
    , m_rotation(0)
    , m_scalar(1.0)
    , m_step(0)
    , m_formStep(0)
    , m_mergeEnable(true)
    , m_highlightColor(0.3, 0.3, 0.8)
{

}

Sculpture::Sculpture(const std::string& filename)
    : Sculpture()
{
    load(filename);
}

Sculpture::Sculpture(const std::shared_ptr<Mesh>& model, double width, double height)
    : Sculpture()
{

    // Position model in the optimal position
    m_width = width;
    scaleToFit(model, height);

    prepareBox();

    modelBody = std::make_shared<Body>(model);

    applyCompositeColors(false);
    print();
}

bool Sculpture::serialize(std::ofstream& file) const
{
    if (CompositeBody::serialize(file)) {

        modelBody->mesh()->serialize(file);

        Serializer::writeDouble(file, m_width);
        Serializer::writeDouble(file, m_height);
        Serializer::writeDouble(file, m_rotation);
        Serializer::writeDouble(file, m_scalar);

        Serializer::writeUint(file, m_step);

        Serializer::writeBool(file, m_mergeEnable);

        Serializer::writeDVec3(file, m_highlightColor);

        return true;
    }

    return false;
}
bool Sculpture::deserialize(std::ifstream& file)
{
    if (CompositeBody::deserialize(file)) {

        modelBody = std::make_shared<Body>(std::make_shared<Mesh>(file));

        m_width = Serializer::readDouble(file);
        m_height = Serializer::readDouble(file);
        m_rotation = Serializer::readDouble(file);
        m_scalar = Serializer::readDouble(file);

        m_step = Serializer::readUint(file);

        m_mergeEnable = Serializer::readBool(file);

        m_highlightColor = Serializer::readDVec3(file);

        return true;
    }

    return false;
}

void Sculpture::scaleToFit(const std::shared_ptr<Mesh>& model, double maxHeight)
{
    // Find the maximum dimensions of the mesh
    double xNear, xFar, yNear, yFar, zNear, zFar;
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
    m_scalar = std::min(m_width / (xFar - xNear), m_width / (zFar - zNear));
    if (m_height * m_scalar > maxHeight) { // Adjust scale to ensure the model does not exceed the maximum allowable height
        m_scalar = maxHeight / m_height;
    }

    m_height *= m_scalar;

    model->scale(m_scalar); // Scale model to fit within specified limits
}

void Sculpture::prepareBox()
{
    m_mesh = MeshBuilder::box(m_width, m_width, m_height);
    m_mesh->translate({ 0, m_height / 2, 0 });
    m_mesh->setBaseColor(baseColor());
    m_hull = ConvexHull(m_mesh);
    CompositeBody::restore();

    prepareTree();
}

void Sculpture::update()
{

}

void Sculpture::restore()
{
    prepareBox();
}
void Sculpture::restoreAsHull()
{
    m_hull = ConvexHull(modelBody->mesh());
    m_mesh = std::make_shared<Mesh>(m_hull);
    m_mesh->setFaceColor(baseColor());
    CompositeBody::restore();
}

void Sculpture::moved()
{
    if (modelBody != nullptr) modelBody->setTransform(m_transform);
}

void Sculpture::reset()
{
    m_operations.clear();
    m_step = 0;
    m_formStep = 0;
}

void Sculpture::queueSection(const Plane& plane)
{
    m_operations.emplace_back(plane.origin, plane.normal);
}

void Sculpture::queueSection(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& normal)
{
    m_operations.emplace_back();

    auto &operation = m_operations[m_operations.size() - 1];
    operation.surfaces.reserve(2);
    operation.limits.reserve(1);

    operation.surfaces.emplace_back(a, glm::normalize(glm::cross(normal, b - a)));
    operation.surfaces.emplace_back(b, glm::normalize(glm::cross(normal, c - b)));

    operation.limits.emplace_back(c, glm::normalize(glm::cross(normal, a - c)));
}

std::shared_ptr<Debris> Sculpture::applySection()
{
    if (m_step < m_operations.size()) {
        uint32_t step = m_step++;

        switch (m_operations[step].surfaces.size()) {
            case 1: return planarSection(m_operations[step].surfaces[0]);
            case 2: return triangleSection(m_operations[step].surfaces[0], m_operations[step].surfaces[1], m_operations[step].limits[0]);
            default:
                std::cout << "[Sculpture] Queued section not handled. Not currently supported\n";
        }
    }

    return nullptr;
}

std::shared_ptr<Debris> Sculpture::planarSection(const Plane& plane)
{
    m_newFaces.clear();

    std::cout << "PS0\n";
    if (!Collision::test(m_hull, plane)) return nullptr;

    auto debris = std::make_shared<Debris>();
    for (uint32_t i = 0; i < components().size(); i++) {
        if (Collision::below(components()[i], plane)) {
            debris->add(components()[i]);
            remove(i);
            i--;
        }
    }

    const std::vector<uint32_t>& sources = split(plane);
    for (uint32_t source : sources) {
        debris->add(components()[components().size() - 1]);
        remove(components().size() - 1);

        // Record index of newly introduced face
        uint32_t index = components()[source].faces().matchFace(plane.normal), faceCount = 0;
        for (uint32_t j = 0; j < source; j++) faceCount += components()[j].facetCount();
        m_newFaces.emplace_back(faceCount + index);
    }

    std::cout << "R " << debris->components().size() << " " << components().size() << "\n";

    if (!debris->components().empty()) { // Action was taken to remove parts of the sculpture
        debris->setTransform(m_transform);
        debris->initialize();

        debris->addFixedPlane(plane);

        if (components().empty()) throw std::runtime_error("[Sculpture] Empty sculpture");
        m_hull = components()[0]; // Ignores small off-cuts (Little effect on testing) & only works properly before triangular sectioning TODO
        remesh();

        return debris;
    }

    return nullptr;
}

std::shared_ptr<Debris> Sculpture::triangleSection(const Plane& planeA, const Plane& planeB, const Plane& limit)
{
    m_newFaces.clear();

    auto debris = std::make_shared<Debris>();

    const auto& fragments = components();
    for (uint32_t i = 0; i < fragments.size(); i++) {

        auto lFragments = Collision::fragments(fragments[i], limit.inverted());

        auto aFragments = Collision::fragments(lFragments.second, planeA.inverted());
        auto bFragments = Collision::fragments(aFragments.second, planeB.inverted());

        if (!bFragments.second.empty()) { // Body is within bounds of cut
            uint32_t idx = i;

            // Attach fragments (outside the triangle) from the cuts back
            if (!lFragments.first.empty()) idx = attach(lFragments.first, idx);
            if (!aFragments.first.empty()) idx = attach(aFragments.first, idx);
            if (!bFragments.first.empty()) idx = attach(bFragments.first, idx);

            // Hull is entirely within the cut - No hull to be replaced, must be removed outright
            if (idx == i) {
                remove(i);
                i--;
            }

            debris->add(bFragments.second);
        }
    }

    if (!debris->components().empty()) { // Action was taken to remove parts of the sculpture
        debris->setTransform(m_transform);
        debris->initialize();

        debris->addFixedPlane(planeA);
        debris->addFixedPlane(planeB);

        // Manually remesh only if no hulls are merged (If they are CompositeBody would remesh)
        if (!m_mergeEnable || !tryMerge()) remesh();
//        remesh();

        return debris;
    }

    return nullptr;
}

uint32_t Sculpture::attach(const ConvexHull& hull, uint32_t index)
{
    if (index >= components().size()) add(hull);
    else replace(hull, index);

    return components().size();
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

void Sculpture::enableHullMerging(bool enable)
{
    m_mergeEnable = enable;
}

void Sculpture::remesh()
{
    CompositeBody::remesh();

    // Provide styling to indicate freshly cut faces
    for (uint32_t face : m_newFaces) {
        m_mesh->setFaceColor(face, m_highlightColor);
    }
}

const std::shared_ptr<Mesh>& Sculpture::sculpture()
{
    return m_mesh;
}

const std::shared_ptr<Body>& Sculpture::model()
{
    return modelBody;
}


double Sculpture::width() const
{
    return m_width;
}
double Sculpture::height() const
{
    return m_height;
}
double Sculpture::rotation() const
{
    return m_rotation;
}

double Sculpture::scalar() const
{
    return m_scalar;
}

double Sculpture::initialVolume() const
{
    return m_width * m_width * m_height;
}
double Sculpture::currentVolume() const
{
    return m_mesh->volume();
}
double Sculpture::finalVolume() const
{
    return modelBody->mesh()->volume();
}

double Sculpture::bulkUsageRatio() const
{
    return finalVolume() / initialVolume();
}
double Sculpture::remainderRatio() const
{
    double fVolume = finalVolume();

    return (currentVolume() - fVolume) / (initialVolume() - fVolume);
}

void Sculpture::print() const
{
    std::cout << "[Sculpture] width: " << m_width << ", height: " << m_height
              << ", volume ratio: " << 100 * bulkUsageRatio() << "% material usage, " << 100 * remainderRatio() << "% remaining\n";

    std::cout << "hulls: " << components().size() << "\n";
}