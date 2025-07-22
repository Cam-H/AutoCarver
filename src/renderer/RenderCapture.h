//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_RENDERCAPTURE_H
#define AUTOCARVER_RENDERCAPTURE_H

#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLPaintDevice>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QColor>

#include <QOpenGLFramebufferObject>
#include <memory>
#include <mutex>

#include <vec3.hpp>

class Mesh;
class RenderGeometry;

#include "Camera.h"


class RenderCapture : public QOffscreenSurface {
public:

    RenderCapture(QScreen *screen, const QSize& size);

    ~RenderCapture();

    void addTarget(const std::shared_ptr<Mesh>& mesh, const QColor& color = QColor(255, 0, 255));
    void clearTargets();

    void resize(int width, int height);
    void resize(const QSize& size);

    void focus();

    void setClearColor(QColor color);

    void capture();

    Camera& camera();

    QImage grabFramebuffer();

protected:

    struct Target {

        Target (const std::shared_ptr<Mesh>& mesh);

        std::shared_ptr<Mesh> mesh;
        uint32_t count;

        QOpenGLBuffer vbo;
        QOpenGLBuffer ibo;

        QColor color;
    };

    void focusScene();
    void focusTarget(const Target& target);

    std::tuple<float, float, float, float> getBounds(const Target& target, const glm::vec3& fwd, const glm::vec3& horz, const glm::vec3& vert);
    static void correctBounds(float& left, float& right, float& bot, float& top);

    void initialize();
    void prepare();

    void makeCurrent();
    void doneCurrent();

    void disable();

    void initializeGL();
//
//    virtual void resizeGL(
//            int width,
//            int height) override;
//
    void paintGL();

    [[nodiscard]] QSize bufferSize() const;
    [[nodiscard]] bool isValid() const;

private:

    std::mutex m_mutex;

    QOpenGLContext* m_context = nullptr;
    QOpenGLFunctions* m_functions = nullptr;
    QOpenGLPaintDevice* m_paintDevice = nullptr;

    QOpenGLShaderProgram* m_program = nullptr;

    QOpenGLFramebufferObject *m_fbo = nullptr;

    bool m_initialized;
    bool m_initializedGL;

    QSize m_size;

    Camera m_camera;

    std::vector<Target> m_targets;
};


#endif //AUTOCARVER_RENDERCAPTURE_H
