//
// Created by Cam on 2025-04-14.
//

#include "Serializer.h"

#include <vector>
#include <iostream>
#include <limits>

#include <gtc/type_ptr.hpp>

Serializer *Serializer::instance = new Serializer();

Serializer::Serializer ()
        : m_OK(false)
{
    ioTest();
}

void Serializer::ioTest()
{
    m_OK = false;

    std::string filename = "serialtest.bin";

    // Generate test material and convert it to bytes
    std::vector<uint32_t> uintTests = {
            0, 1, 555,
//            std::numeric_limits<uint32_t>::max(), 0x80000000,
            0x80, 0x8000, 0x800000
    };

    std::streamsize size = uintTests.size() * sizeof(uint32_t);
    char *input = new char[size], *ptr = input;

    for (uint32_t test : uintTests) {
        toBytes(test, ptr);
        ptr += sizeof(uint32_t);
    }

    // Serialize the test material
    if (serialize(filename, input, size)) {

        // Deserialize the test material
        char* output = deserialize(filename, size);
        if (output != nullptr) {
            m_OK = true;

            // Compare bytes of input & output

            // Compare test material directly
            ptr = output;
            for (uint32_t test : uintTests) {
                uint32_t result = toUint(ptr);
                if (test != result) {
                    std::cerr << "De/Serialization test failed:\n";
                    printBits(test);
                    printBits(result);
                    m_OK = false;
                    break;
                }

                ptr += sizeof(uint32_t);
            }
        }

        delete[] output;
    }

    delete[] input;
}

bool Serializer::serialize(const std::string& filename, const char* buffer, std::streamsize size)
{
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file for test writing.\n";
        return false;
    }

    file.write(buffer, size);

    file.close();
    return true;
}

char* Serializer::deserialize(const std::string& filename, std::streamsize size)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file for test reading\n";
        return nullptr;
    }

    char *buffer = new char[size];
    file.read(buffer, size);

    file.close();
    return buffer;
}

Serializer* Serializer::getInstance()
{
    return instance;
}

bool Serializer::isOK()
{
    return m_OK;
}

bool Serializer::writeBool(std::ofstream& file, bool value)
{
    file.write((const char*)&value, sizeof(bool));
    return true;
}

bool Serializer::writeUint(std::ofstream& file, uint32_t value)
{
    char *buffer = new char[4];
    toBytes(value, buffer);

    file.write(buffer, sizeof(uint32_t));

    delete[] buffer;
    return true;
}

bool Serializer::writeDVec2(std::ofstream& file, const glm::dvec2& value)
{
    file.write(reinterpret_cast<const char*>(glm::value_ptr(value)), sizeof(glm::dvec2));
    return true;
}

bool Serializer::writeDVec3(std::ofstream& file, const glm::dvec3& value)
{
    file.write(reinterpret_cast<const char*>(glm::value_ptr(value)), sizeof(glm::dvec3));
    return true;
}

bool Serializer::writeVectorDVec2(std::ofstream& file, const std::vector<glm::dvec2>& values)
{
    Serializer::writeUint(file, values.size());
    file.write(reinterpret_cast<const char*>(values.data()), values.size() * sizeof(glm::dvec2));
    return true;
}

bool Serializer::writeVectorDVec3(std::ofstream& file, const std::vector<glm::dvec3>& values)
{
    Serializer::writeUint(file, values.size());
    file.write(reinterpret_cast<const char*>(values.data()), values.size() * sizeof(glm::dvec3));
    return true;
}

bool Serializer::writeTransform(std::ofstream& file, const glm::dmat4x4& value)
{
    file.write(reinterpret_cast<const char*>(glm::value_ptr(value)), sizeof(glm::dmat4x4));
    return true;
}

void Serializer::toBytes(int value, char* buffer)
{
//    memcpy(buffer, &value, 4);
//    buffer[0] = value;
//    buffer[1] = value >> 8;
//    buffer[2] = value >> 16;
//    buffer[3] = value >> 24;

    buffer[0] = (unsigned char)((value      ) & 0xFF);
    buffer[1] = (unsigned char)((value >>  8) & 0xFF);
    buffer[2] = (unsigned char)((value >> 16) & 0xFF);
    buffer[3] = (unsigned char)((value >> 24) & 0xFF);
}

void Serializer::toBytes(uint32_t value, char* buffer)
{
    toBytes((int)value, buffer);
}

bool Serializer::readBool(std::ifstream& file)
{
    char* buffer = new char[1];

    file.read(buffer, sizeof(bool));
    bool data = *buffer;

    delete[] buffer;

    return data;
}

uint32_t Serializer::readUint(std::ifstream& file)
{
    char* buffer = new char[4];

    file.read(buffer, sizeof(uint32_t));
    uint32_t data = toUint(buffer);

    delete[] buffer;

    return data;
}

glm::dvec2 Serializer::readDVec2(std::ifstream& file)
{
    auto *buffer = new double[2];
    file.read(reinterpret_cast<char*>(buffer), sizeof(glm::dvec2));

    auto vector = glm::make_vec2(buffer);
    delete[] buffer;

    return vector;
}

glm::dvec3 Serializer::readDVec3(std::ifstream& file)
{
    auto *buffer = new double[3];
    file.read(reinterpret_cast<char*>(buffer), sizeof(glm::dvec3));

    auto vector = glm::make_vec3(buffer);
    delete[] buffer;

    return vector;
}

std::vector<glm::dvec2> Serializer::readVectorDVec2(std::ifstream& file)
{
    uint32_t size = readUint(file);
    std::vector<glm::dvec2> vertices(size);

    file.read(reinterpret_cast<char*>(vertices.data()), size * sizeof(glm::dvec2));

    return vertices;
}

std::vector<glm::dvec3> Serializer::readVectorDVec3(std::ifstream& file)
{
    uint32_t size = readUint(file);
    std::vector<glm::dvec3> vertices(size);

    file.read(reinterpret_cast<char*>(vertices.data()), size * sizeof(glm::dvec3));

    return vertices;
}

glm::dmat4x4 Serializer::readTransform(std::ifstream& file)
{
    auto *buffer = new float[16];
    file.read(reinterpret_cast<char*>(buffer), sizeof(glm::dmat4x4));

    auto transform = glm::make_mat4(buffer); // TODO check
    delete[] buffer;

    return transform;
}


int Serializer::toInt(const char* buffer)
{
    int data = ((unsigned char)buffer[3] << 24) | ((unsigned char)buffer[2] << 16) | ((unsigned char)buffer[1] << 8) | ((unsigned char)buffer[0]);// TODO care with endian
//    int data = (buffer[3]) | (buffer[2]. << 8) | (buffer[1] << 16) | (buffer[0] << 24);
    return data;
}

uint32_t Serializer::toUint(const char* buffer)
{
    return (uint32_t) toInt(buffer); // TODO proper uint read
}

void Serializer::printBits(uint32_t value)
{
    for (uint32_t i = 0; i < 32; i++) {
        std::cout << (value & (0x80000000 >> i) ? "1" : "0");
        if (i % 8 == 7) std::cout << " ";
    }
    std::cout << " " << value << "\n";
}