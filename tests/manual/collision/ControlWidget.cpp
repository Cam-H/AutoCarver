//
// Created by Cam on 2025-04-02.
//

#include "ControlWidget.h"
#include "geometry/MeshBuilder.h"
#include "geometry/EPA.h"

ControlWidget::ControlWidget(Scene* scene, QWidget* parent)
        : SceneWidget(scene, parent)
{

    auto sphere = MeshBuilder::icosphere(0.08f);
    sphere->setBaseColor({0, 0, 1});

    m_points.push_back(scene->createBody(sphere));

    sphere = MeshBuilder::icosphere(0.08f);
    sphere->setBaseColor({0, 1, 0});

    m_points.push_back(scene->createBody(sphere));
}

void ControlWidget::keyPressEvent(QKeyEvent *e)
{
    if (m_scene == nullptr || m_scene->bodyCount() < 2) return;

    float delta = 0.05f, rDel = M_PI / 64, theta = 0;
    glm::vec3 offset = {};
    switch (e->key()) {
        case Qt::Key::Key_Up:
            offset += glm::vec3(0, delta, 0);
            break;
        case Qt::Key::Key_Down:
            offset += glm::vec3(0, -delta, 0);
            break;
        case Qt::Key::Key_W:
            offset += glm::vec3(0, 0, delta);
            break;
        case Qt::Key::Key_S:
            offset += glm::vec3(0, 0, -delta);
            break;
        case Qt::Key::Key_A:
            offset += glm::vec3(delta, 0, 0);
            break;
        case Qt::Key::Key_D:
            offset += glm::vec3(-delta, 0, 0);
            break;
        case Qt::Key::Key_Q:
            theta += rDel;
            break;
        case Qt::Key::Key_E:
            theta -= rDel;
            break;
    }

    uint32_t idx1 = !QGuiApplication::keyboardModifiers().testFlag(Qt::ShiftModifier), idx2 = (idx1 == 0);

    if (glm::length(offset) > 0 || theta != 0) {
        m_scene->bodies()[idx1]->globalTranslate(offset);
        if (theta != 0) m_scene->bodies()[idx1]->globalRotate(glm::vec3{1, 0, 0}, theta);

        EPA result = m_scene->bodies()[0]->collision(m_scene->bodies()[1]);
        if (result.colliding()) {
            m_scene->bodies()[idx2]->translate((idx1 ? 1.0f : -1.0f) * result.overlap());
        }

        m_scene->bodies()[idx1]->mesh()->setBaseColor(result.colliding() ? glm::vec3{1, 0, 0} : glm::vec3{1, 1, 1});
        updateRenderGeometry(m_scene->bodies()[idx1]->mesh());
        m_scene->bodies()[idx2]->mesh()->setBaseColor(result.colliding() ? glm::vec3{0.6, 0, 0} : glm::vec3{1, 1, 1});
        updateRenderGeometry(m_scene->bodies()[idx2]->mesh());

        m_points[0]->setPosition(result.colliderAClosest());
        m_points[1]->setPosition(result.colliderBClosest());

        update();
    }

}