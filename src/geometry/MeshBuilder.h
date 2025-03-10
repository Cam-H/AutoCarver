//
// Created by Cam on 2024-10-25.
//

#ifndef AUTOCARVER_MESHBUILDER_H
#define AUTOCARVER_MESHBUILDER_H

#include "Mesh.h"

#include "VertexArray.h"
#include "FaceArray.h"

#include <vector>

class MeshBuilder {
public:

    static std::shared_ptr<Mesh> plane(float width, const vec3f& origin, const vec3f& normal);
//    static std::shared_ptr<Mesh> plane(float length, float width, const vec3f& origin, const vec3f& normal, const vec3f& ref);
    static std::shared_ptr<Mesh> plane(float length, float width, const vec3f& origin, const vec3f& normal, const vec3f& ref);

    static std::shared_ptr<Mesh> box(float length, float width, float height);

    static std::shared_ptr<Mesh> eliminateCoincidentVertices(const std::shared_ptr<Mesh>& mesh);

    static std::shared_ptr<Mesh> cleaned(const std::shared_ptr<Mesh>& mesh);
    static std::shared_ptr<Mesh> cleaned(const VertexArray& vertices, const FaceArray& faces);

    static bool isManifold(const std::shared_ptr<Mesh>& mesh);
    static bool isManifold(const FaceArray& faces);

private:

    static void eliminateCoincidentVertices(const FaceArray& srcFaces, std::vector<vec3f>& vertices, std::vector<std::vector<uint32_t>>& faces);

    static std::shared_ptr<Mesh> cleaned(std::vector<vec3f>& vertices, const std::vector<vec3f>& normals, const FaceArray& faces);

    [[nodiscard]] static size_t hash(const vec3f& vec, float factor);
    [[nodiscard]] static size_t hash(size_t a, size_t b, size_t c) ;
    [[nodiscard]] static size_t cantor(size_t a, size_t b) ;

    static std::vector<std::vector<uint32_t>> identifyNeighbors(const FaceArray& faces);
    static uint64_t linkKey(uint32_t I0, uint32_t I1);
};


#endif //AUTOCARVER_MESHBUILDER_H
