//
// Created by Cam on 2025-04-02.
//

#include "ControlWidget.h"
#include "geometry/MeshBuilder.h"
#include "geometry/EPA.h"
#include "geometry/Collision.h"
#include "geometry/shape/AABB.h"

ControlWidget::ControlWidget(const std::shared_ptr<Scene>& scene, QWidget* parent)
        : SceneWidget(scene, parent)
{

    auto sphere = MeshBuilder::icosphere(0.08f);
    sphere->setBaseColor({0, 0, 1});
    scene->createBody(sphere);

    sphere = MeshBuilder::icosphere(0.08f);
    sphere->setBaseColor({0, 1, 0});
    scene->createBody(sphere);

//    for (uint32_t i = 0; i < sphere->faceCount(); i += 2) {
//        sphere->setFaceColor(i, {1, 0, 1});
//    }
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
        case Qt::Key::Key_Space:
            handleCollision();
            return;
    }

    uint32_t idx1 = !QGuiApplication::keyboardModifiers().testFlag(Qt::ShiftModifier), idx2 = (idx1 == 0);

    if (glm::length(offset) > 0 || theta != 0) {
        m_scene->bodies()[idx1]->globalTranslate(offset);
        if (theta != 0) m_scene->bodies()[idx1]->rotate(glm::vec3{1, 0, 0}, theta);

        handleCollision(idx2);
    }
}

void ControlWidget::handleCollision(uint32_t active)
{
//    auto center = m_scene->bodies()[1]->position() - m_scene->bodies()[0]->position();
////    auto body = Sphere{ center, 1.0f };
//    auto body = AABB{ center - glm::vec3{ 0.5f, 0.5f, 0.5f}, center + glm::vec3{ 0.5f, 0.5f, 0.5f}};
//    auto result = Collision::test(m_scene->bodies()[0]->hull(), body);
//    std::cout << "Intersection: " << result << "\n";

    EPA result = m_scene->bodies()[0]->collision(m_scene->bodies()[1]);
    if (result.colliding()) {
        m_scene->bodies()[active]->translate((!active ? 1.0f : -1.0f) * result.overlap());
        std::cout << "Overlap: " << result.overlap().x << " " << result.overlap().y << " " << result.overlap().z << "\n";
    }

    m_scene->bodies()[!active]->mesh()->setBaseColor(result.colliding() ? glm::vec3{1, 0, 0} : glm::vec3{1, 1, 1});
    updateRenderGeometry(m_scene->bodies()[!active]->mesh());
    m_scene->bodies()[active]->mesh()->setBaseColor(result.colliding() ? glm::vec3{0.6, 0, 0} : glm::vec3{1, 1, 1});
    updateRenderGeometry(m_scene->bodies()[active]->mesh());

//    m_scene->bodies()[2]->setPosition(result.colliderAClosest());
//    m_scene->bodies()[3]->setPosition(result.colliderBClosest());

    update();
}