//
// Created by Cam on 2024-10-25.
//

#ifndef AUTOCARVER_MESHBUILDER_H
#define AUTOCARVER_MESHBUILDER_H


#include "VertexArray.h"
#include "FaceArray.h"

#include <vector>
#include <map>

class Triangle;
class Plane;
class Axis3D;
class Mesh;
class ConvexHull;
class Octree;

class MeshBuilder {
public:

    static std::shared_ptr<Mesh> plane(const Plane& obj, double width);
    static std::shared_ptr<Mesh> plane(const Axis3D& system, const glm::dvec3& origin, double length, double width);

    static std::shared_ptr<Mesh> box(double sideLength = 1.0);
    static std::shared_ptr<Mesh> box(double length, double width, double height);

    static std::shared_ptr<Mesh> cylinder(double radius = 1.0, double height = 1.0, uint32_t segments = 6);
    static std::shared_ptr<Mesh> cylinder(const glm::dvec3& axis, double radius = 1.0, uint32_t segments = 6);

    static std::shared_ptr<Mesh> extrude(const std::vector<glm::dvec3>& border, const glm::dvec3& normal, double depth = 1.0);

    static std::shared_ptr<Mesh> icosahedron(double radius = 1.0);
    static std::shared_ptr<Mesh> icosphere(double radius = 1.0, uint8_t subdivisions = 1);

    static std::shared_ptr<Mesh> axes(const Axis3D& system);

    static std::shared_ptr<Mesh> mesh(const std::shared_ptr<Octree>& tree);

    static std::shared_ptr<Mesh> merge(const std::shared_ptr<Mesh>& a, const std::shared_ptr<Mesh>& b);
    static std::shared_ptr<Mesh> merge(const std::vector<std::shared_ptr<Mesh>>& meshes);

    static std::shared_ptr<Mesh> composite(const std::vector<ConvexHull>& hulls);

//    static std::shared_ptr<Mesh> eliminateCoincidentVertices(const std::shared_ptr<Mesh>& mesh);

//    static std::shared_ptr<Mesh> cleaned(const std::shared_ptr<Mesh>& mesh);
//    static std::shared_ptr<Mesh> cleaned(const VertexArray& vertices, const FaceArray& faces);


    static bool isManifold(const std::shared_ptr<Mesh>& mesh);
    static bool isManifold(const FaceArray& faces);

private:

    static void icosahedron(double radius, std::vector<glm::dvec3>& vertices, std::vector<TriIndex>& faces);
    static uint32_t getMidPoint(std::vector<glm::dvec3>& vertices, std::map<uint64_t, uint32_t> &table, uint64_t iA, uint64_t iB, double scalar);

//    static std::shared_ptr<Mesh> cube(glm::dvec3 *vertexPtr, uint32_t *facePtr);

    static void indexBox(uint32_t *facePtr, uint32_t *sizePtr, uint32_t offset = 0);

    static void eliminateCoincidentVertices(const FaceArray& srcFaces, std::vector<glm::dvec3>& vertices, std::vector<std::vector<uint32_t>>& faces);

    static std::shared_ptr<Mesh> cleaned(std::vector<glm::dvec3>& vertices, const std::vector<glm::dvec3>& normals, const FaceArray& faces);

    [[nodiscard]] static size_t hash(const glm::dvec3& vec, double factor);
    [[nodiscard]] static size_t hash(size_t a, size_t b, size_t c) ;
    [[nodiscard]] static size_t cantor(size_t a, size_t b) ;

};


#endif //AUTOCARVER_MESHBUILDER_H
