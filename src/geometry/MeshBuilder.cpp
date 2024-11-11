//
// Created by Cam on 2024-10-25.
//

#include "MeshBuilder.h"

std::shared_ptr<Mesh> MeshBuilder::box(float length, float width, float height){

    width /=2;
    length /= 2;
    height /=2;

    auto vertices = new float[24] {
        -length, -height, -width,
        -length,  height, -width,
         length,  height, -width,
         length, -height, -width,
        -length, -height,  width,
        -length,  height,  width,
         length,  height,  width,
         length, -height,  width
    };

    auto indices = new uint32_t[36] {
            0, 1, 2,
            2, 3, 0,
            0, 4, 5,
            5, 1, 0,
            1, 5, 6,
            6, 2, 1,
            2, 6, 7,
            7, 3, 2,
            3, 7, 4,
            4, 0, 3,
            4, 7, 6,
            6, 5, 4
    };

    return std::make_shared<Mesh>(vertices, 8, indices, 12);

}
