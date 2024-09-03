//
// Created by cameronh on 25/04/24.
//

#ifndef AUTOCARVER_SCENE_H
#define AUTOCARVER_SCENE_H

#include <QtCore/QObject>
#include <QVector3D>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>

#include <Qt3DRender/QMesh>
#include <Qt3DCore/QBuffer>
#include <Qt3DLogic/QFrameAction>

#include "../geometry/Tesselation.h"
#include <vector>

class Scene : public QObject
{
    Q_OBJECT

public:
    explicit Scene(Qt3DCore::QEntity *rootEntity);
    ~Scene();


public slots:
    void apexX(int value);
    void apexY(int value);
    void apexZ(int value);
    void cut(int value);
    void show(bool enabled);

private:
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DRender::QMesh *m_BunnyMesh;
    Tesselation m_tessel;

    Qt3DCore::QTransform *m_cutPlaneTransform;
    Qt3DCore::QEntity *m_plane;

    std::vector<QVector3D> m_set;
    QVector3D m_apex;
    bool good;

    Qt3DLogic::QFrameAction *m_frameAction;

};


#endif //AUTOCARVER_SCENE_H
