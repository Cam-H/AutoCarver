//
// Created by cameronh on 24/04/24.
//

#ifndef AUTOCARVER_DATAHANDLER_H
#define AUTOCARVER_DATAHANDLER_H

#include <vector>
#include <string>
#include <memory>

#include <QVector3D>

#include "../geometry/Tesselation.h"

class DataHandler {
public:

    DataHandler();

    virtual ~DataHandler() = default;

    virtual std::shared_ptr<Tesselation> loadTesselation(const std::string &filepath) = 0;

protected:

    static uint32_t nextUInt(std::vector<uint8_t>::const_iterator it);

    static float nextFloat(std::vector<uint8_t>::const_iterator it);

    static QVector3D nextVec3(std::vector<uint8_t>::const_iterator it);

    static QVector3D castVec3(const std::string &vec, char delim);

    static QVector3D castVec3(const std::string &vec, const std::string &delim);

    static std::vector<std::string> split(const std::string &content, char delim);

    static std::vector<std::string> split(const std::string &content, const std::string &delim);

    static void ltrim(std::string &s);

    static void rtrim(std::string &s);

    static void trim(std::string &s);

};

#endif //AUTOCARVER_DATAHANDLER_H
