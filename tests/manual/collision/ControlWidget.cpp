//
// Created by Cam on 2025-04-02.
//

#include "ControlWidget.h"

void ControlWidget::keyPressEvent(QKeyEvent *e)
{
    if (m_scene == nullptr || m_scene->bodyCount() < 2) return;

    float delta = 0.05f;
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
    }

    if (glm::length(offset) > 0) {
        m_scene->bodies()[1]->mesh()->translate(offset.x, offset.y, offset.z);
        m_scene->bodies()[1]->updateHull();


        bool result = m_scene->bodies()[0]->collision(m_scene->bodies()[1], offset);
        std::cout << offset.x << " " << offset.y << " " << offset.z << "\n";
        if (result) {
            m_scene->bodies()[0]->mesh()->translate(offset.x, offset.y, offset.z);
            m_scene->bodies()[0]->updateHull();
            updateRenderGeometry(m_scene->bodies()[0]->mesh());

        }

        m_scene->bodies()[1]->mesh()->setBaseColor(result ? glm::vec3{1, 0, 0} : glm::vec3{1, 1, 1});

        updateRenderGeometry(m_scene->bodies()[1]->mesh());

        update();
    }

}