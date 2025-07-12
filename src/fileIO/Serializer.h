//
// Created by Cam on 2025-04-14.
//

#ifndef AUTOCARVER_SERIALIZER_H
#define AUTOCARVER_SERIALIZER_H

#include <string>
#include <fstream>

#include <glm.hpp>

class Serializer {
private:

    bool m_OK;

    static Serializer *instance;

    Serializer();

    void ioTest();
    static bool serialize(const std::string& filename, const char* buffer, std::streamsize size);
    static char* deserialize(const std::string& filename, std::streamsize size);


public:

    Serializer(const Serializer& obj) = delete;

    static Serializer* getInstance();

    bool isOK();

    static bool writeBool(std::ofstream& file, bool value);
    static bool writeUint(std::ofstream& file, uint32_t value);

    static bool writeVec2(std::ofstream& file, const glm::vec2& value);
    static bool writeVec3(std::ofstream& file, const glm::vec3& value);

    static bool writeVectorVec2(std::ofstream& file, const std::vector<glm::vec2>& values);
    static bool writeVectorVec3(std::ofstream& file, const std::vector<glm::vec3>& values);
    static bool writeTransform(std::ofstream& file, const glm::mat4x4& value);

    static bool readBool(std::ifstream& file);
    static uint32_t readUint(std::ifstream& file);

    static glm::vec2 readVec2(std::ifstream& file);
    static glm::vec3 readVec3(std::ifstream& file);

    static std::vector<glm::vec2> readVectorVec2(std::ifstream& file);
    static std::vector<glm::vec3> readVectorVec3(std::ifstream& file);


    static glm::mat4x4 readTransform(std::ifstream& file);

    static void toBytes(int value, char* buffer);
    static void toBytes(uint32_t value, char* buffer);

    static int toInt(const char* buffer);
    static uint32_t toUint(const char* buffer);

    // Debugging method to check bit patterns
    static void printBits(uint32_t value);
};

#endif //AUTOCARVER_SERIALIZER_H
