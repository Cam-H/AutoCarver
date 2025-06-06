//
// Created by Cam on 2025-03-15.
//

#include "RenderCapture.h"

#include <QtGui/QImage>

#include <iostream>
#include <mutex>
#include <QPainter>

#include "geometry/Mesh.h"
#include "RenderGeometry.h"

RenderCapture::RenderCapture(QScreen *screen, const QSize& size)
    : QOffscreenSurface(screen)
    , m_size(size)
    , m_initialized(false)
    , m_initializedGL(false)
    , m_camera()
{

    setFormat(QSurfaceFormat::defaultFormat());

    create();  // TODO care

    initialize();
}

void RenderCapture::initialize()
{
    if (!m_initialized) {
        m_initialized = true;

        m_context = new QOpenGLContext(this);
        m_context->setFormat(format());

        if (m_context->create()) {
            m_context->makeCurrent(this);
            // initialize the OpenGL 2.1 / ES 2.0 functions for this object
            m_functions = m_context->functions();
            m_functions->initializeOpenGLFunctions();
            // try initializing the OpenGL 3.0 functions for this object
//            m_functions_3_0 = m_context->versionFunctions <QOpenGLFunctions_3_0>();
//            if (m_functions_3_0) {
//                m_functions_3_0->initializeOpenGLFunctions();
//            } else {
//                // if we do not have OpenGL 3.0 functions, glBlitFrameBuffer is not available, so we
//                // must do the blit
//                // using a shader and the framebuffer texture, so we need to create the shader
//                // here...
//                // --> allocate m_blitShader, a simple shader for drawing a textured quad
//                // --> build quad geometry, VBO, whatever
//            }
            // now we have a context, create the FBO
            prepare();
        } else {
            m_initialized = false;
            delete m_context;
            m_context = nullptr;
            throw ("Failed to create OpenGL context!");
        }
    }
}

void RenderCapture::prepare()
{
    if (m_context && (m_fbo == nullptr || m_fbo->size() != bufferSize())) {
        m_context->makeCurrent(this);

        // free old FBOs
        if (m_fbo) {
            m_fbo->release();
            delete m_fbo;
            m_fbo = nullptr;
        }

        QOpenGLFramebufferObjectFormat format;
        format.setSamples(0);

        m_fbo = new QOpenGLFramebufferObject(bufferSize(), format);
        if (!m_fbo->isValid()) {
            throw ("OpenGlOffscreenSurface::recreateFbo() - Failed to create background FBO!");
        }
        // clear framebuffer
        m_fbo->bind();
        m_functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        m_fbo->release();
    }

    // create paint device for painting with QPainter if needed
    if (!m_paintDevice) {
        m_paintDevice = new QOpenGLPaintDevice;
    }

    std::cout << bufferSize().width() << " " << m_paintDevice->size().width() << " " << m_size.width() << " A\n";

    // update paint device size if needed
    if (m_paintDevice->size() != bufferSize()) {
        m_paintDevice->setSize(bufferSize());
    }
}

RenderCapture::~RenderCapture()
{
    // to delete the FBOs we first need to make the context current
    m_context->makeCurrent(this);
    // destroy framebuffer objects
    if (m_fbo) {
        m_fbo->release();
        delete m_fbo;
        m_fbo = nullptr;
    }

    // free context
    m_context->doneCurrent();
    delete m_context;
    m_context = nullptr;
    // free paint device
    delete m_paintDevice;
    m_paintDevice = nullptr;
    m_initialized = m_initializedGL = false;

    destroy();
}

void RenderCapture::addTarget(const std::shared_ptr<Mesh>& mesh, const QColor& color)
{
    m_targets.emplace_back(mesh);
    Target& target = m_targets[m_targets.size() - 1];

    target.vbo.create();
    target.ibo.create();

    target.vbo.bind();
    target.ibo.bind();

    target.vbo.allocate(mesh->vertices().data(), 3 * mesh->vertexCount() * sizeof(float));
    target.ibo.allocate(mesh->indices(), target.count * sizeof(uint32_t));

    target.color = color;
}
void RenderCapture::clearTargets()
{
    for (Target& target : m_targets) {
        target.vbo.destroy();
        target.ibo.destroy();
    }

    m_targets.clear();
}

void RenderCapture::resize(int width, int height)
{
    resize(QSize(width, height));
}

void RenderCapture::resize(const QSize& size)
{
    m_mutex.lock();

    m_size = size;

    makeCurrent();

    prepare();

    m_mutex.unlock();

}

void RenderCapture::focus()
{
    QVector3D horz = m_camera.horizontal(), vert = m_camera.vertical();

    glm::vec3 hAxis = { horz.x(), horz.y(), horz.z() }, vAxis = { vert.x(), vert.y(), vert.z() };

    // Find maximum extents of the targets along the camera view axes
    float left = 1e6, right = -1e6, bot = 1e6, top = -1e6;
    for (const Target& target : m_targets) {
        float min, max;

        target.mesh->extents(hAxis, min, max);
        if (min < left) left = min;
        if (max > right) right = max;

        target.mesh->extents(vAxis, min, max);
        if (min < bot) bot = min;
        if (max > top) top = max;
    }

    // Square bounds - Prevent distortion of the render
    float width = right - left, height = top - bot;
    if (width < height) {
        float delta = (height - width) / 2;
        left -= delta;
        right += delta;
        width = height;
    } else if (height < width) {
        float delta = (width - height) / 2;
        bot -= delta;
        top += delta;
    }

    // Apply a margin so the render does not touch edges
    float margin = 0.1f * width;
    left -= margin;
    right += margin;
    bot -= margin;
    top += margin;

    // Ensure that the targets are centered in the screen
    QVector3D center = horz * (left + right) / 2 + vert * (bot + top) / 2;
    m_camera.setCenter(center);

    m_camera.setRect(left, right, bot, top);
}

void RenderCapture::setClearColor(QColor color)
{
    m_functions->glClearColor(color.redF(), color.greenF(), color.blueF(), color.alphaF());
}

void RenderCapture::capture()
{
    std::lock_guard <std::mutex> locker(m_mutex);

    initialize();

    makeCurrent();
    if (!m_initializedGL) {
        m_initializedGL = true;
        initializeGL();
    }

    if (m_fbo) m_fbo->bind();
    else QOpenGLFramebufferObject::bindDefault();

    paintGL();
    doneCurrent();
}

void RenderCapture::disable()
{
    //TODO
}

void RenderCapture::initializeGL()
{
    setClearColor(Qt::black);
    m_functions->glViewport(0, 0, m_size.width(), m_size.height());

    m_program = new QOpenGLShaderProgram(m_context);

    if (!m_program->addShaderFromSourceFile(QOpenGLShader::Vertex, R"(..\res\shaders\pure.vert)"))
        disable();

    if (!m_program->addShaderFromSourceFile(QOpenGLShader::Fragment, R"(..\res\shaders\pure.frag)"))
        disable();

    if (!m_program->link()) disable();

    m_program->bind();

}

void RenderCapture::makeCurrent()
{
    if (isValid()) {
        m_context->makeCurrent(this);
    } else {
        throw ("OpenGlOffscreenSurface::makeCurrent() - Window not yet properly initialized!");
    }
}

void RenderCapture::doneCurrent()
{
    if (m_context) {
        m_context->doneCurrent();
    }
}

Camera& RenderCapture::camera()
{
    return m_camera;
}

QImage RenderCapture::grabFramebuffer()
{
    std::lock_guard <std::mutex> locker(m_mutex);
    makeCurrent();

    QImage image;
    // bind framebuffer first
    m_functions->glBindFramebuffer(GL_READ_FRAMEBUFFER, m_fbo->handle());
//    if (m_functions_3_0) m_functions_3_0->glReadBuffer(GL_COLOR_ATTACHMENT0);

    GLenum internalFormat = m_fbo->format().internalTextureFormat();
    bool hasAlpha = internalFormat == GL_RGBA || internalFormat == GL_BGRA || internalFormat == GL_RGBA8;
    if (internalFormat == GL_BGRA) {
        image = QImage(m_fbo->size(), hasAlpha ? QImage::Format_ARGB32 : QImage::Format_RGB32);
        m_functions->glReadPixels(0, 0, m_fbo->size().width(),
                                  m_fbo->size().height(), GL_BGRA, GL_UNSIGNED_BYTE, image.bits());
    } else if ((internalFormat == GL_RGBA) || (internalFormat == GL_RGBA8)) {
        image = QImage(m_fbo->size(), hasAlpha ? QImage::Format_RGBA8888 : QImage::Format_RGBX8888);
        m_functions->glReadPixels(0, 0, m_fbo->size().width(),
                                  m_fbo->size().height(), GL_RGBA, GL_UNSIGNED_BYTE, image.bits());
    } else {
        qDebug() << "OpenGlOffscreenSurface::grabFramebuffer() - Unsupported framebuffer format"
                 << internalFormat << "!";
    }
    m_functions->glBindFramebuffer(GL_FRAMEBUFFER, m_fbo->handle());

    return (image.mirrored());
}

void RenderCapture::paintGL()
{

    QPainter painter(m_paintDevice);

    painter.beginNativePainting();

    // Clear color and depth buffer
    m_functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable depth buffer
    m_functions->glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    m_functions->glEnable(GL_CULL_FACE);


    m_program->bind();

    m_program->setUniformValue("mvp_matrix", m_camera.getViewProjection());


    for (Target& target : m_targets) {
        m_program->setUniformValue("out_color", target.color);

        target.vbo.bind();
        target.ibo.bind();

        m_program->enableAttributeArray(0);
        m_program->setAttributeBuffer(0, GL_FLOAT, 0, 3, 3 * sizeof(float));

        m_functions->glDrawElements(GL_TRIANGLES, target.count, GL_UNSIGNED_INT, nullptr);

    }

//    m_program->release();

    painter.endNativePainting();


    painter.end();
}

QSize RenderCapture::bufferSize() const
{
    return m_size;
}

bool RenderCapture::isValid() const
{
    return m_initialized && m_context && m_fbo;
}

RenderCapture::Target::Target (const std::shared_ptr<Mesh>& mesh)
    : mesh(mesh)
    , count(3 * mesh->triangleCount())
    , vbo(QOpenGLBuffer::VertexBuffer)
    , ibo(QOpenGLBuffer::IndexBuffer)
    , color(Qt::magenta)
{

}