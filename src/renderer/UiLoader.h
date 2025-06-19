//
// Created by Cam on 2025-04-17.
//

#ifndef AUTOCARVER_UILOADER_H
#define AUTOCARVER_UILOADER_H

#include <QtWidgets>
#include <QUiLoader>

#include "SceneWidget.h"
#include "LineChartWidget.h"
#include "PolygonWidget.h"

class UiLoader : public QUiLoader {
public:

    UiLoader(QObject *parent = nullptr) : QUiLoader(parent) {}

    QWidget *createWidget(const QString& className, QWidget *parent = nullptr, const QString& name = QString())
    {
        if (className == "SceneWidget") {
            auto *w = new SceneWidget(parent);
            w->setObjectName(name);
            return w;
        } else if (className == "LineChartWidget") {
            auto *w = new LineChartWidget(parent);
            w->setObjectName(name);
            return w;
        } else if (className == "PolygonWidget") {
            auto *w = new PolygonWidget(parent);
            w->setObjectName(name);
            return w;
        }

        return QUiLoader::createWidget(className, parent, name);
    }
};


#endif //AUTOCARVER_UILOADER_H