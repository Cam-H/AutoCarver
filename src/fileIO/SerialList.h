//
// Created by cjhat on 2025-10-03.
//

#ifndef AUTOCARVER_SERIALLIST_H
#define AUTOCARVER_SERIALLIST_H

#include "Serializable.h"
#include <vector>

// Provides functionality to save a list of Serializable objects
template<class T>
class SerialList : public Serializable {
public:
    explicit SerialList(const std::vector<T>& objects) : objects(objects) {}
    explicit SerialList(const std::string& filename)
    {
        Serializable::load(filename);
    }

    explicit SerialList(std::ifstream& file)
    {
        SerialList::deserialize(file);
    }

    bool serialize(std::ofstream& file) const override
    {
        Serializer::writeUint(file, objects.size());
        for (const T& obj : objects) obj.serialize(file);
        return true;
    }

    bool deserialize(std::ifstream& file) override
    {
        uint32_t count = Serializer::readUint(file);
        for (uint32_t i = 0; i < count; i++) {
            objects.emplace_back(file);
        }

        return true;
    }

public:
    std::vector<T> objects;
};


#endif //AUTOCARVER_SERIALLIST_H