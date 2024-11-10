//
// Created by Cam on 2024-11-09.
//

#ifndef AUTOCARVER_RENDERENTITY_H
#define AUTOCARVER_RENDERENTITY_H

// Rendering
#include <QColor>
#include <QVector3D>
#include <QQuaternion>
#include <QGeometry>

#include <Qt3DCore/QEntity>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DCore/QTransform>
#include <QGeometryRenderer>


#include <vector>

#include "geometry/Mesh.h"

class RenderEntity : Qt3DCore::QEntity {
public:

//    class Configuration {
//    public:
//        Configuration(Qt3DRender::QGeometryRenderer *render, Qt3DRender::QMaterial *material);
//
//    public:
//        Qt3DRender::QGeometryRenderer *render;
//        Qt3DRender::QMaterial *material;
//        Qt3DCore::QTransform *transform;
//        Qt3DCore::QEntity *entity;
//    };

    RenderEntity(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view);

    void show(uint32_t idx);
    void hide(uint32_t idx);

    void setTranslation(QVector3D translation);
    void setRotation(QQuaternion rotation);
    Qt3DCore::QTransform *transformation();

    void add(const std::shared_ptr<Mesh>& mesh);

    void generate();

private:

    static Qt3DCore::QGeometry* indexedGeometry(const std::shared_ptr<Mesh>& mesh, Qt3DCore::QNode *parent);
    static Qt3DCore::QGeometry* faceGeometry(const std::shared_ptr<Mesh>& mesh, Qt3DCore::QNode *parent);

private:

    Qt3DExtras::Qt3DWindow *view;

    Qt3DCore::QTransform *m_transform;

    std::vector<std::shared_ptr<Mesh>> meshes;
    std::vector<Qt3DCore::QEntity*> m_renders;
};


#endif //AUTOCARVER_RENDERENTITY_H
