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

    bool save(const std::string& filename) const;
    bool load(const std::string& filename);

protected:

    virtual bool serialize(std::ofstream& file) const = 0;
    virtual bool deserialize(std::ifstream& file) = 0;
};


#endif //AUTOCARVER_SERIALIZABLE_H
