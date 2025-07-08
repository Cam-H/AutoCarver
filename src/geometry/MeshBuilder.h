//
// Created by Cam on 2024-10-25.
//

#ifndef AUTOCARVER_MESHBUILDER_H
#define AUTOCARVER_MESHBUILDER_H

#include "Mesh.h"

#include "VertexArray.h"
#include "FaceArray.h"
#include "Triangle.h"

#include <vector>
#include <map>

class MeshBuilder {
public:

    static std::shared_ptr<Mesh> plane(float width, const glm::vec3& origin, const glm::vec3& normal);
//    static std::shared_ptr<Mesh> plane(float length, float width, const vec3f& origin, const vec3f& normal, const vec3f& ref);
    static std::shared_ptr<Mesh> plane(float length, float width, const glm::vec3& origin, const glm::vec3& normal, const glm::vec3& ref);

    static std::shared_ptr<Mesh> box(float sideLength = 1.0f);
    static std::shared_ptr<Mesh> box(float length, float width, float height);

    static std::shared_ptr<Mesh> cylinder(float radius = 1.0f, float height = 1.0f, uint32_t segments = 6);
    static std::shared_ptr<Mesh> extrude(const std::vector<glm::vec3>& border, const glm::vec3& normal, float depth = 1.0f);

    static std::shared_ptr<Mesh> icosahedron(float radius = 1.0f);
    static std::shared_ptr<Mesh> icosphere(float radius = 1.0f, uint8_t subdivisions = 1);


    static std::shared_ptr<Mesh> merge(const std::shared_ptr<Mesh>& a, const std::shared_ptr<Mesh>& b);

    static std::shared_ptr<Mesh> composite(const std::vector<ConvexHull>& hulls);

    static std::shared_ptr<Mesh> eliminateCoincidentVertices(const std::shared_ptr<Mesh>& mesh);

    static std::shared_ptr<Mesh> cleaned(const std::shared_ptr<Mesh>& mesh);
    static std::shared_ptr<Mesh> cleaned(const VertexArray& vertices, const FaceArray& faces);

    static bool isManifold(const std::shared_ptr<Mesh>& mesh);
    static bool isManifold(const FaceArray& faces);

private:

    static void icosahedron(float radius, std::vector<glm::vec3>& vertices, std::vector<Triangle>& faces);
    static uint32_t getMidPoint(std::vector<glm::vec3>& vertices, std::map<uint64_t, uint32_t> &table, uint64_t iA, uint64_t iB, float scalar);

    static void eliminateCoincidentVertices(const FaceArray& srcFaces, std::vector<glm::vec3>& vertices, std::vector<std::vector<uint32_t>>& faces);

    static std::shared_ptr<Mesh> cleaned(std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals, const FaceArray& faces);

    [[nodiscard]] static size_t hash(const glm::vec3& vec, float factor);
    [[nodiscard]] static size_t hash(size_t a, size_t b, size_t c) ;
    [[nodiscard]] static size_t cantor(size_t a, size_t b) ;

};


#endif //AUTOCARVER_MESHBUILDER_H
