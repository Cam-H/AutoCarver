//
// Created by Cam on 2025-04-13.
//

#ifndef AUTOCARVER_SERIALIZABLE_H
#define AUTOCARVER_SERIALIZABLE_H

#include <fstream>
#include <string>

#include "Serializer.h"

class Serializable {
public:

//    virtual ~Serializable() = default;

    virtual bool serialize(const std::string& filename);
    virtual bool serialize(std::ofstream& file) = 0;

    virtual bool deserialize(const std::string& filename);
    virtual bool deserialize(std::ifstream& file) = 0;
};


#endif //AUTOCARVER_SERIALIZABLE_H
