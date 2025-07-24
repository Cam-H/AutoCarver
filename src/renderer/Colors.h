//
// Created by cjhat on 2025-07-13.
//

#ifndef AUTOCARVER_COLORS_H
#define AUTOCARVER_COLORS_H

#include <array>
#include <vec3.hpp>

static glm::vec3 rgb(uint32_t hexcode) {
    return {
            (double)((hexcode & 0xFF0000) >> 16) / 255,
            (double)((hexcode & 0xFF00) >> 8) / 255,
            (double)((hexcode & 0xFF)) / 255
    };
}

static std::array<glm::vec3, 8> PURE_COLORS = {
        glm::vec3{ 1.0f, 0.0f, 0.0f},
        glm::vec3{ 0.0f, 1.0f, 0.0f},
        glm::vec3{ 1.0f, 1.0f, 0.0f},
        glm::vec3{ 0.0f, 0.0f, 1.0f},
        glm::vec3{ 1.0f, 0.0f, 1.0f},
        glm::vec3{ 0.0f, 1.0f, 1.0f},
        glm::vec3{ 1.0f, 1.0f, 1.0f},
        glm::vec3{ 0.0f, 0.0f, 0.0f},
};

static std::array<glm::vec3, 30> DULL_SET = {
        rgb(0x450C1C),
        rgb(0x2E251A),
        rgb(0x133F4A),
        rgb(0x800020),
        rgb(0x690D0B),
        rgb(0x294E90),
        rgb(0x311440),
        rgb(0x7D0D2C),
        rgb(0x4F3F28),
        rgb(0x22797A),
        rgb(0x695A46),
        rgb(0xC20044),
        rgb(0xE91815),
        rgb(0x3C91ED),
        rgb(0x572671),
        rgb(0xA8092D),
        rgb(0x72B01D),
        rgb(0x40E0D0),
        rgb(0xD2B48C),
        rgb(0xFF7CA7),
        rgb(0xDAD0D0),
        rgb(0x7EBDE9),
        rgb(0xF1EDF3),
        rgb(0xD2042D),
        rgb(0x3F7D20),
        rgb(0x6FFFE9),
        rgb(0xE9DAC6),
        rgb(0xF5E7E7),
        rgb(0xF3F0F0),
        rgb(0xA3D1CC)
};

static std::array<glm::vec3, 30> BRIGHT_SET = {
        rgb(0xFF2F28),
        rgb(0xF40000),
        rgb(0xBA0203),
        rgb(0x7F0405),
        rgb(0xF8F9FA),
        rgb(0xCFD1D4),
        rgb(0xA5A9AE),
        rgb(0x5F6266),
        rgb(0xCD9C20),
        rgb(0xF5Cb5C),
        rgb(0xF2ECDD), // 10
        rgb(0xD500FF),
        rgb(0xAB00C0),
        rgb(0x800080),
        rgb(0x1F822E),
        rgb(0x30B852),
        rgb(0xDAFFDA),
        rgb(0xFFD9DA),
        rgb(0xEB638B),
        rgb(0xAC274F),
        rgb(0xE85D04), // 20
        rgb(0xDC2F02),
        rgb(0xD00000),
        rgb(0xF5F5DC),
        rgb(0xDED1B6),
        rgb(0xC6AC8F),
        rgb(0x6FFFE9),
        rgb(0x40E0D0),
        rgb(0x22797A),
        rgb(0xC8FF00)
};

#endif //AUTOCARVER_COLORS_H
