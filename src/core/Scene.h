//
// Created by cameronh on 25/04/24.
//

#ifndef AUTOCARVER_SCENE_H
#define AUTOCARVER_SCENE_H

#include <QtCore/QObject>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>

#include <Qt3DRender/QMesh>
#include <Qt3DCore/QBuffer>
#include <Qt3DLogic/QFrameAction>

#include "../geometry/Tesselation.h"

class Scene : public QObject
{
    Q_OBJECT

public:
    explicit Scene(Qt3DCore::QEntity *rootEntity);
    ~Scene();

public slots:

private:
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DRender::QMesh *m_BunnyMesh;
    Tesselation m_tessel;
    bool good;

    Qt3DLogic::QFrameAction *m_frameAction;

};


#endif //AUTOCARVER_SCENE_H
