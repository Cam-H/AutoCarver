//
// Created by Cam on 2025-03-15.
//

#include "RenderCapture.h"

#include <iostream>

void RenderCapture::capture()
{
    std::cout << "Request capture\n";
    if (!m_reply) {
        std::cout << "Try capture\n";
        m_reply = m_capture->requestCapture();
        connection = QObject::connect(m_reply,
                                      &Qt3DRender::QRenderCaptureReply::completed,
                                      this,
                                      &RenderCapture::onCompletion);
    }
}

void RenderCapture::onCompletion()
{
    std::cout << "Result captured\n";

    QObject::disconnect(connection);
    m_imageLabel->setPixmap(QPixmap::fromImage(m_reply->image()));
//    m_reply->saveImage(
//            "capture.bmp");
    m_reply->deleteLater();
    m_reply = nullptr;
    if (m_continuous)
        capture();
}