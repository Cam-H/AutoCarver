//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_RENDERCAPTURE_H
#define AUTOCARVER_RENDERCAPTURE_H

#include <Qt3DRender/QRenderCapture>
#include <QLabel>


class RenderCapture : public QObject {
Q_OBJECT
public:
    RenderCapture(Qt3DRender::QRenderCapture* capture, QLabel *imageLabel)
    : m_capture(capture)
    , m_reply(nullptr)
    , m_imageLabel(imageLabel)
    , m_continuous(false)
    {
    }

    void capture();

public slots:
    void onCompletion();
private:
    Qt3DRender::QRenderCapture* m_capture;
    Qt3DRender::QRenderCaptureReply *m_reply;
    QMetaObject::Connection connection;
    QLabel *m_imageLabel;
    bool m_continuous;
};


#endif //AUTOCARVER_RENDERCAPTURE_H
