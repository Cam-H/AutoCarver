//
// Created by Cam on 2024-11-10.
//

#include "Sculpture.h"

#include "geometry/MeshBuilder.h"

Sculpture::Sculpture(const std::shared_ptr<Mesh> &model, float width, float height)
    : RigidBody(MeshBuilder::box(width, width, height))
    , model(model)
    , m_width(width)
    , m_height(height)
{

    scaleToFit(m_width, m_height);

    m_mesh->scale({ 1, m_height / height, 1 });
    m_mesh->translate({ 0, m_height / 2, 0 });

    std::cout << "Volume ratio: " << 100 * bulkUsageRatio() << "% material usage, " << 100 * remainderRatio() << "% Remaining\n";
}

//void Sculpture::setRenderer(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view)
//{
//    m_render = new RenderEntity(parent, view);
//    m_render->add(m_mesh);
//
//    auto material = new Qt3DExtras::QDiffuseSpecularMaterial();
//    material->setShininess(0);
//    material->setDiffuse(QColor::fromRgbF(0.2f, 0.2f, 0.2f, 1));
//    material->setAmbient(QColor::fromRgbF(0, 0, 0.8f, 1));
//
//    m_render->add(m_sculpture, material);
//
////    if (m_hullOK) m_render->add(std::make_shared<Mesh>(m_hull));
//
//    updateRenderer();
//}

void Sculpture::scaleToFit(float width, float maxHeight)
{
    // Find the maximum dimensions of the mesh
    float xNear, xFar, yNear, yFar, zNear, zFar;
    model->xExtents(xNear, xFar);
    model->yExtents(yNear, yFar);
    model->zExtents(zNear, zFar);

    model->translate({ -(xNear + xFar) / 2, -yNear, -(zNear + zFar) / 2 }); // Center the model in the center
    model->rotate({ 0, 1, 0 }, atanf((xFar - xNear) / (zFar - zNear))); // Rotate the model to make best use of diagonal space

    // Re-evaluate extents to figure out scaling requirements
    model->xExtents(xNear, xFar);
    model->zExtents(zNear, zFar);

    model->translate({ -(xNear + xFar) / 2, 0, -(zNear + zFar) / 2 }); // Adjust model in the center

    m_height = yFar - yNear;

    // Calculate the scale factor to make the mesh as large as possible, within the specified limits
    float scalar = std::min(width / (xFar - xNear), width / (zFar - zNear));
    if (m_height * scalar > maxHeight) {
        scalar = maxHeight / m_height;
    }

    m_height *= scalar;
//    m_height = maxHeight;

    model->scale(scalar); // Scale model to fit within specified limits
}

void Sculpture::update()
{
//    m_mesh->rotate(0, 1, 0, 0.001f);

}

// TODO review fit
void Sculpture::overrwrite(const std::shared_ptr<Mesh>& sculpture)
{
    m_mesh = sculpture;
}

const std::shared_ptr<Mesh>& Sculpture::sculpture()
{
    return m_mesh;
}


float Sculpture::width() const
{
    return m_width;
}
float Sculpture::height() const
{
    return m_height;
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
    return model->volume();
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