//
// Created by cameronh on 24/04/24.
//

#include "DataHandler.h"

#include <sstream>
#include <algorithm>
#include <cstring>
#include <iostream>

DataHandler::DataHandler()
{
}

uint32_t DataHandler::nextUInt(std::vector<uint8_t>::const_iterator it)
{
    uint32_t value;
    uint8_t buffer[] = {*it, *(it + 1), *(it + 2), *(it + 3)};
    memcpy(&value, &buffer, sizeof(value));
    return value;
}

float DataHandler::nextFloat(std::vector<uint8_t>::const_iterator it)
{
    float value;
    uint8_t buffer[] = {*it, *(it + 1), *(it + 2), *(it + 3)};
    memcpy(&value, &buffer, sizeof(value));
    return value;
}

QVector3D DataHandler::nextVec3(std::vector<uint8_t>::const_iterator it)
{
    return {nextFloat(it), nextFloat(it + 4), nextFloat(it + 8)};
}

QVector3D DataHandler::castVec3(const std::string& vec, char delim)
{
    uint16_t first = vec.find(delim);
    uint16_t last = vec.find_last_of(delim);

    return QVector3D{
            (float)atof(vec.substr(0, first).c_str()),
            (float)atof(vec.substr(first, last).c_str()),
            (float)atof(vec.substr(last).c_str())
    };
}

QVector3D DataHandler::castVec3(const std::string& vec, const std::string& delim)
{
    uint16_t first = vec.find(delim);
    uint16_t last = vec.find_last_of(delim);

    return {
            (float)atof(vec.substr(0, first).c_str()),
            (float)atof(vec.substr(first + delim.length(), last - first - delim.length()).c_str()),
            (float)atof(vec.substr(last).c_str())
    };
}

std::vector<std::string> DataHandler::split(const std::string& content, char delim)
{
    std::vector<std::string> splits;

    std::istringstream f(content);
    std::string str;

    while(std::getline(f, str, delim)) {
        splits.push_back(str);
    }

    return splits;
}

std::vector<std::string> DataHandler::split(const std::string& content, const std::string& delim)
{
    std::vector<std::string> splits;

    size_t current = 0;
    size_t next = content.find(delim);
    while(next != std::string::npos){
        splits.push_back(content.substr(current, next - current));

        current = next + delim.length();
        next = content.find(delim, current);
    }

    if(current > 0 && current < content.length()){
        splits.push_back(content.substr(current));
    }

    return splits;
}

void DataHandler::ltrim(std::string &s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char c){
        return !std::isspace(c);
    }));
}

void DataHandler::rtrim(std::string &s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char c) {
        return !std::isspace(c);
    }).base(), s.end());
}

void DataHandler::trim(std::string &s)
{
    rtrim(s);
    ltrim(s);
}