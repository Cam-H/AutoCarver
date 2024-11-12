//
// Created by cameronh on 24/04/24.
//

#include "FileHandler.h"

#include <sys/stat.h>
#include <algorithm>

bool FileHandler::fileExists(const std::string& filepath) {
    struct stat buffer;

    return stat(filepath.c_str(), &buffer) == 0;
}

FileFormat FileHandler::getFileFormat(const std::string& filepath) {
    std::string fileFormat = filepath.substr(filepath.find_last_of('.') + 1);

    // Convert file extension to lowercase
    std::transform(fileFormat.begin(), fileFormat.end(), fileFormat.begin(), [](unsigned char c) { return std::tolower(c); });

    if(fileFormat == "obj") {
        return FileFormat::OBJ;
    }else if (fileFormat == "mtl") {
        return FileFormat::MTL;
    }else if(fileFormat == "stl"){
        return FileFormat::STL;
    }else if(fileFormat == "ply"){
        return FileFormat::PLY;
    }else if(fileFormat == "step"){
        return FileFormat::STEP;
    }else if(fileFormat == "sldprt"){
        return FileFormat::SLDPRT;
    }else if(fileFormat == "sldasm"){
        return FileFormat::SLDASM;
    }

    return FileFormat::NONE;
}

std::string FileHandler::getFileName(const std::string& filepath){
    auto it = filepath.find_last_of('/');
    if(it != std::string::npos){
        return filepath.substr(filepath.find_last_of('/') + 1);
    }

    return filepath;
}