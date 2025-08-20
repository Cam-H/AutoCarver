//
// Created by Cam on 2025-04-13.
//

#include "Serializable.h"

#include <iostream>

bool Serializable::save(const std::string& filename) const
{
    if (!Serializer::getInstance()->isOK()) {
        std::cerr << "Issues with serializer. Object serialization is not available\n";
        return false;
    }

    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file for writing.\n";
        return false;
    }

    bool success = serialize(file);

    file.close();

    if (success) std::cout << "Object serialized successfully at: [" << filename << "]\n";
    else std::cerr << "Failed to serialize the object\n";

    return success;
}

bool Serializable::load(const std::string& filename)
{
    if (!Serializer::getInstance()->isOK()) {
        std::cerr << "Issues with serializer. Object deserialization is not available\n";
        return false;
    }

    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file for reading\n";
        return false;
    }

    bool success = deserialize(file);

    file.close();

    if (success) std::cout << "Object deserialized successfully\n";
    else std::cerr << "Failed to deserialize the object\n";

    return success;
}