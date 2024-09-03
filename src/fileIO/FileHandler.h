//
// Created by cameronh on 24/04/24.
//

#ifndef AUTOCARVER_FILEHANDLER_H
#define AUTOCARVER_FILEHANDLER_H

#include <string>
#include <vector>

#include "../geometry/Tesselation.h"

enum class FileFormat
{
    NONE = 0, OBJ, MTL, STL, PLY, STEP, SLDPRT, SLDASM
};

class FileHandler
{
public:

    static bool fileExists(const std::string& filepath);
    static FileFormat getFileFormat(const std::string& filepath);
    static std::string getFileName(const std::string& filepath);


};


#endif //AUTOCARVER_FILEHANDLER_H
