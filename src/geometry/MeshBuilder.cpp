//
// Created by Cam on 2024-10-25.
//

#include "MeshBuilder.h"

std::shared_ptr<Mesh> MeshBuilder::plane(float width, const vec3f& origin, const vec3f& normal)
{
    vec3f ref = normal.y == 1 ? vec3f{1, 0, 0} : vec3f{0, 1, 0};
    return plane(width, width, origin, normal, ref);
}

std::shared_ptr<Mesh> MeshBuilder::plane(float length, float width, const vec3f& origin, const vec3f& normal, const vec3f& ref)
{

    vec3f wAxis = normal.cross(ref).normalized(), lAxis = normal.cross(wAxis).normalized();
    vec3f a = origin + wAxis * width / 2 + lAxis * length / 2, b = a - wAxis * width, c = b - lAxis * length, d = c + wAxis * width;


    auto vertices = new float[12] {
        a.x, a.y, a.z,
        b.x, b.y, b.z,
        c.x, c.y, c.z,
        d.x, d.y, d.z
    };

    auto faces = new uint32_t[4] { 0, 1, 2, 3 };
    auto faceSizes = new uint32_t[1] { 4 };

    return std::make_shared<Mesh>(vertices, 4, faces, faceSizes, 1);
}

std::shared_ptr<Mesh> MeshBuilder::box(float length, float width, float height)
{
    length /= 2;
    width /=2;
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
