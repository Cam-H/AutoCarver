//
// Created by cameronh on 25/04/24.
//

#ifndef AUTOCARVER_NOTSCENE_H
#define AUTOCARVER_NOTSCENE_H

#include <QtCore/QObject>
#include <QVector3D>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <QMouseEvent>


#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QText2DEntity>

#include <Qt3DRender/QCamera>
#include <Qt3DRender/QMesh>
#include <Qt3DCore/QBuffer>
#include <Qt3DLogic/QFrameAction>

#include "../geometry/Body.h"
#include "../geometry/Tesselation.h"
#include <vector>

class NotScene : public Qt3DExtras::Qt3DWindow
{
    Q_OBJECT

public:
    explicit NotScene(Qt3DCore::QEntity *rootEntity, const std::string& base);
    ~NotScene();


public slots:
    void apexX(int value);
    void apexY(int value);
    void apexZ(int value);
    void cut(int value);
    void show(bool enabled);

protected:
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DRender::QMesh *m_BunnyMesh;
    std::string m_filepath;

    Body* m_body;
    Tesselation m_tessel;

    Qt3DCore::QTransform *m_cutPlaneTransform;
    Qt3DCore::QTransform *m_cuboidTransform;

    Qt3DCore::QEntity *m_plane;

    std::vector<QVector3D> m_set;
    QVector3D m_apex;
    bool good;

    Qt3DExtras::QText2DEntity *m_text;
    Qt3DCore::QTransform *m_textTransform;

    Qt3DLogic::QFrameAction *m_frameAction;
};


#endif //AUTOCARVER_NOTSCENE_H
