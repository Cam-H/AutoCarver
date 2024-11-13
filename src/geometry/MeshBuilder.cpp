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

    auto faces = new uint32_t[24] {
            0, 1, 2, 3,
            0, 4, 5, 1,
            0, 3, 7, 4,
            6, 5, 4, 7,
            6, 2, 1, 5,
            6, 7, 3, 2
    };

    auto faceSizes = new uint32_t[6] { 4, 4, 4, 4, 4, 4 };

    return std::make_shared<Mesh>(vertices, 8, faces, faceSizes, 6);

}
