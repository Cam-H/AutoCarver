//
// Created by Cam on 2024-10-25.
//

#include "MeshBuilder.h"

#include <glm.hpp>
#include <array>
#include <unordered_map>

#include "Mesh.h"
#include "geometry/primitives/Plane.h"
#include "geometry/primitives/Triangle.h"
#include "geometry/primitives/ConvexHull.h"
#include "Axis3D.h"
#include "Octree.h"

#include "renderer/Colors.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/quaternion.hpp>


std::shared_ptr<Mesh> MeshBuilder::plane(const Plane& obj, double width)
{
    return plane(Axis3D(obj.normal), obj.origin, width, width);
}

std::shared_ptr<Mesh> MeshBuilder::plane(const Axis3D& system, const glm::dvec3& origin, double length, double width)
{

    glm::dvec3 a = origin + system.xAxis * width * 0.5 + system.yAxis * length * 0.5,
               b = a - system.xAxis * width,
               c = b - system.yAxis * length,
               d = c + system.xAxis * width;

    auto mesh = std::make_shared<Mesh>(4, 2, 8);

    mesh->m_vertices[0] = { a.x, a.y, a.z };
    mesh->m_vertices[1] = { b.x, b.y, c.z };
    mesh->m_vertices[2] = { c.x, c.y, b.z };
    mesh->m_vertices[3] = { d.x, d.y, d.z };

    uint32_t *idxPtr = mesh->m_faces[0], *sizePtr = mesh->m_faces.faceSizes();

    *idxPtr++ = 0;
    *idxPtr++ = 1;
    *idxPtr++ = 2;
    *idxPtr++ = 3;

    *idxPtr++ = 0;
    *idxPtr++ = 3;
    *idxPtr++ = 2;
    *idxPtr++ = 1;

    *sizePtr++ = 4;
    *sizePtr++ = 4;

    mesh->initialize();

    return mesh;
}

std::shared_ptr<Mesh> MeshBuilder::box(double sideLength)
{
    return box(sideLength, sideLength, sideLength);
}

std::shared_ptr<Mesh> MeshBuilder::box(double length, double width, double height)
{
    length /= 2;
    width /=2;
    height /=2;

    auto mesh = std::make_shared<Mesh>(8, 6, 24);

    mesh->m_vertices[0] = { -length, -height, -width };
    mesh->m_vertices[1] = { -length,  height, -width };
    mesh->m_vertices[2] = {  length,  height, -width };
    mesh->m_vertices[3] = {  length, -height, -width };
    mesh->m_vertices[4] = { -length, -height,  width };
    mesh->m_vertices[5] = { -length,  height,  width };
    mesh->m_vertices[6] = {  length,  height,  width };
    mesh->m_vertices[7] = {  length, -height,  width };

    uint32_t *idxPtr = mesh->m_faces[0], *sizePtr = mesh->m_faces.faceSizes();
    indexBox(idxPtr, sizePtr);

    mesh->initialize();

    return mesh;
}

std::shared_ptr<Mesh> MeshBuilder::cylinder(double radius, double height, uint32_t segments){

    std::vector<glm::dvec3> border(segments);

    double theta = 0;
    const double increment = 2.0 * (M_PI / segments);
    for (uint32_t i = 0; i < segments; i++) {
        border[i] = {radius * cos(theta), height, radius * sin(theta) };
        theta += increment;
    }

    return extrude(border, { 0, -1, 0 }, height);
}

std::shared_ptr<Mesh> MeshBuilder::cylinder(const glm::dvec3& axis, double radius, uint32_t segments)
{
    std::vector<glm::dvec3> border(segments);

    double length = glm::length(axis);
    Axis3D axes(axis / length);

    glm::dquat rotation = glm::angleAxis(2.0 * (M_PI / segments), axes.zAxis);
    border[0] = axes.xAxis * radius + axis;

    for (uint32_t i = 1; i < segments; i++) {
        border[i] = border[i - 1] * rotation;
    }

    return extrude(border, -axis, length);
}

std::shared_ptr<Mesh> MeshBuilder::extrude(const std::vector<glm::dvec3>& border, const glm::dvec3& normal, double depth){
    if (border.size() < 3) return nullptr;

    uint32_t segments = border.size(), idx = 0;
    uint32_t vertexCount = 2 * border.size(), faceCount = 2 + segments, indexCount = vertexCount + 4 * segments;

    auto mesh = std::make_shared<Mesh>(vertexCount, faceCount, indexCount);

    for (const glm::dvec3& vertex : border) {
        mesh->m_vertices[idx++] = {
                vertex.x + normal.x * depth,
                vertex.y + normal.y * depth,
                vertex.z + normal.z * depth
        };

        mesh->m_vertices[idx++] = {
                vertex.x,
                vertex.y,
                vertex.z
        };
    }

    uint32_t *idxPtr = mesh->m_faces[0], *sizePtr = mesh->m_faces.faceSizes();

    // Generate flat faces
    for(uint32_t i = 0; i < vertexCount; i+=2) *idxPtr++ = i;
    for(uint32_t i = 1; i < vertexCount; i+=2) *idxPtr++ = vertexCount - i;

    *sizePtr++ = vertexCount / 2;
    *sizePtr++ = vertexCount / 2;

    // Generate border faces
    for (uint32_t i = 0; i < segments; i++) {
        *idxPtr++ = 2 * i;
        *idxPtr++ = 2 * i + 1;
        *idxPtr++ = (2 * i + 3) % (2 * segments);
        *idxPtr++ = (2 * i + 2) % (2 * segments);
        *sizePtr++ = 4;
    }

    mesh->initialize(); // TODO skip and write indices directly (Easily known so can avoid CDT calculation)

    return mesh;
}

std::shared_ptr<Mesh> MeshBuilder::icosahedron(double radius){
    std::vector<glm::dvec3> vertices;
    std::vector<TriIndex> faces;

    icosahedron(radius, vertices, faces);

    return std::make_shared<Mesh>(vertices, faces);
}

void MeshBuilder::icosahedron(double radius, std::vector<glm::dvec3>& vertices, std::vector<TriIndex>& faces)
{

    vertices.reserve(12);
    faces.reserve(20);

    // Prepare icosahedron vertices
    auto t = (double)(1 + sqrt(5) / 2);

    vertices.emplace_back(-1, t, 0);
    vertices.emplace_back(1, t, 0);
    vertices.emplace_back(-1, -t, 0);
    vertices.emplace_back(1, -t, 0);

    vertices.emplace_back(0, -1, t);
    vertices.emplace_back(0, 1, t);
    vertices.emplace_back(0, -1, -t);
    vertices.emplace_back(0, 1, -t);

    vertices.emplace_back(t, 0, -1);
    vertices.emplace_back(t, 0, 1);
    vertices.emplace_back(-t, 0, -1);
    vertices.emplace_back(-t, 0, 1);

    // Bring all vertices on to the unit sphere and scale by parameter
    for (glm::dvec3& vertex : vertices) vertex = radius * glm::normalize(vertex);

    // Triangulate the icosahedron
    faces.emplace_back(0,  11, 5);
    faces.emplace_back(0,  5,  1);
    faces.emplace_back(0,  1,  7);
    faces.emplace_back(0,  7,  10);
    faces.emplace_back(0,  10, 11);

    faces.emplace_back(1,  5,  9);
    faces.emplace_back(5,  11, 4);
    faces.emplace_back(11, 10, 2);
    faces.emplace_back(10, 7,  6);
    faces.emplace_back(7,  1,  8);

    faces.emplace_back(3, 9, 4);
    faces.emplace_back(3, 4, 2);
    faces.emplace_back(3, 2, 6);
    faces.emplace_back(3, 6, 8);
    faces.emplace_back(3, 8, 9);

    faces.emplace_back(4, 9, 5);
    faces.emplace_back(2, 4, 11);
    faces.emplace_back(6, 2, 10);
    faces.emplace_back(8, 6, 7);
    faces.emplace_back(9, 8, 1);
}


std::shared_ptr<Mesh> MeshBuilder::icosphere(double radius, uint8_t subdivisions)
{
    uint32_t vertexCount = 10 * (uint32_t)pow(2, 2 * subdivisions) + 2;

    std::vector<glm::dvec3> vertices;
    vertices.reserve(vertexCount);

    std::vector<TriIndex> faces;
//    faces.reserve(20 * 4 * subdivisions);

    icosahedron(radius, vertices, faces);


//    std::cout << vertexCount << " " << vertices.size() << " " << faces.size()  << " " << subdivisions << "\n";

    // Refine the icosahedron through subdivision of faces
    std::map<uint64_t, uint32_t> table;
    for(uint8_t i = 0; i < subdivisions; i++){
        uint32_t triangleCount = faces.size();

        for(uint32_t j = 0; j < triangleCount; j++){
            uint32_t a = getMidPoint(vertices, table, faces[j].I0, faces[j].I1, radius);
            uint32_t b = getMidPoint(vertices, table, faces[j].I1, faces[j].I2, radius);
            uint32_t c = getMidPoint(vertices, table, faces[j].I2, faces[j].I0, radius);

            faces.emplace_back(faces[j].I0, a, c);
            faces.emplace_back(faces[j].I1, b, a);
            faces.emplace_back(faces[j].I2, c, b);
            faces[j] = TriIndex(a, b, c);
        }
    }

    return std::make_shared<Mesh>(vertices, faces);
}

uint32_t MeshBuilder::getMidPoint(std::vector<glm::dvec3>& vertices, std::map<uint64_t, uint32_t> &table, uint64_t iA, uint64_t iB, double scalar){

    uint64_t key = iA > iB ? (iA << 32) + iB : (iB << 32) + iA;

    auto it = table.find(key);
    if(it == table.end()){
        vertices.push_back(scalar * glm::normalize(vertices[iA] + vertices[iB]));
        table[key] = vertices.size() - 1;
        return vertices.size() - 1;
    }

    return table[key];
}

std::shared_ptr<Mesh> MeshBuilder::axes(const Axis3D& system)
{
    double radius = 0.02f, length = 1.0f;

    auto xAxis = cylinder(system.xAxis * length, radius, 12);
    auto yAxis = cylinder(system.yAxis * length, radius, 12);
    auto zAxis = cylinder(system.zAxis * length, radius, 12);
    auto origin = icosphere(2 * radius, 1);

    xAxis->setFaceColor(RED);
    yAxis->setFaceColor(GREEN);
    zAxis->setFaceColor(BLUE);
    origin->setFaceColor(WHITE);

    return merge({ xAxis, yAxis, zAxis, origin });
}

std::shared_ptr<Mesh> MeshBuilder::mesh(const std::shared_ptr<Octree>& tree)
{
    const uint8_t target = Octree::Octant::Status::TERMINUS;
    const uint32_t  count = tree->octantCount(target);
    uint32_t offset = 0;
    std::cout << count << " / " << tree->maximumOctantCount() << " COUNTS\n";

    auto mesh = std::make_shared<Mesh>(8 * count, 6 * count, 24 * count);

    uint32_t *idxPtr = mesh->m_faces[0], *sizePtr = mesh->m_faces.faceSizes();

    for (const Octree::Octant& octant : *tree) {
        if (octant.status == target) {
            double length = tree->octantLength(octant);
            mesh->m_vertices[offset    ] = octant.top;
            mesh->m_vertices[offset + 1] = octant.top + glm::dvec3{      0, length,      0 };
            mesh->m_vertices[offset + 2] = octant.top + glm::dvec3{ length, length,      0 };
            mesh->m_vertices[offset + 3] = octant.top + glm::dvec3{ length,      0,      0 };
            mesh->m_vertices[offset + 4] = octant.top + glm::dvec3{      0,      0, length };
            mesh->m_vertices[offset + 5] = octant.top + glm::dvec3{      0, length, length };
            mesh->m_vertices[offset + 6] = octant.top + glm::dvec3{ length, length, length };
            mesh->m_vertices[offset + 7] = octant.top + glm::dvec3{ length,      0, length };

            indexBox(idxPtr, sizePtr, offset);
            idxPtr += 24;
            sizePtr += 6;

            offset += 8;
        }
    }

    // Apply unique colors to each face for clarity
    for (uint32_t i = 0; i < 6 * count; i += 6) {
        for (uint8_t j = 0; j < 6; j++) {
            mesh->m_faces.setColor(i + j, PURE_COLORS[j]);
        }
    }

//    std::cout << "done\n";

    mesh->initialize();

    return mesh;
}

void MeshBuilder::indexBox(uint32_t *facePtr, uint32_t *sizePtr, uint32_t offset)
{
    *facePtr++ = 0 + offset;
    *facePtr++ = 1 + offset;
    *facePtr++ = 2 + offset;
    *facePtr++ = 3 + offset;

    *facePtr++ = 0 + offset;
    *facePtr++ = 4 + offset;
    *facePtr++ = 5 + offset;
    *facePtr++ = 1 + offset;

    *facePtr++ = 0 + offset;
    *facePtr++ = 3 + offset;
    *facePtr++ = 7 + offset;
    *facePtr++ = 4 + offset;

    *facePtr++ = 6 + offset;
    *facePtr++ = 5 + offset;
    *facePtr++ = 4 + offset;
    *facePtr++ = 7 + offset;

    *facePtr++ = 6 + offset;
    *facePtr++ = 2 + offset;
    *facePtr++ = 1 + offset;
    *facePtr++ = 5 + offset;

    *facePtr++ = 6 + offset;
    *facePtr++ = 7 + offset;
    *facePtr++ = 3 + offset;
    *facePtr++ = 2 + offset;

    for (uint8_t i = 0; i < 6; i++) *sizePtr++ = 4;
}

std::shared_ptr<Mesh> MeshBuilder::merge(const std::shared_ptr<Mesh>& a, const std::shared_ptr<Mesh>& b)
{
    return merge({ a, b });
}

std::shared_ptr<Mesh> MeshBuilder::merge(const std::vector<std::shared_ptr<Mesh>>& meshes)
{
    if (meshes.empty()) return nullptr;

    // Count the number of features required
    uint32_t vertexCount = 0, faceCount = 0, indexCount = 0, vIdx = 0, vertexOffset = 0;
    for (const std::shared_ptr<Mesh>& mesh : meshes) {
        vertexCount += mesh->vertexCount();
        faceCount += mesh->faceCount();
        indexCount += mesh->faces().indexCount();
    }

    auto newMesh = std::make_shared<Mesh>(vertexCount, faceCount, indexCount);
    uint32_t *idxPtr = newMesh->m_faces[0], *sizePtr = newMesh->m_faces.faceSizes();

    for (const std::shared_ptr<Mesh>& mesh : meshes) {

        // Copy vertex data
        for (const glm::dvec3& vertex : mesh->vertices().vertices()) newMesh->m_vertices[vIdx++] = vertex;

        // Copy face data
        for (uint32_t idx : mesh->faces().faces()) *idxPtr++ = idx + vertexOffset;
        for (uint32_t idx : mesh->faces().faceSizes()) *sizePtr++ = idx;

        vertexOffset += mesh->vertexCount();
    }

    vIdx = 0;

    // Copy colors
    newMesh->setBaseColor(meshes[0]->baseColor());
    for (const std::shared_ptr<Mesh>& mesh : meshes) {
        if (mesh->faceColorsAssigned()) {
            for (uint32_t i = 0; i < mesh->faceCount(); i++) {
                newMesh->setFaceColor(vIdx++, mesh->faces().color(i));
            }
        } else {
            for (uint32_t i = 0; i < mesh->faceCount(); i++) {
                newMesh->setFaceColor(vIdx++, mesh->baseColor());
            }
        }
    }

    newMesh->initialize();

    return newMesh;
}

std::shared_ptr<Mesh> MeshBuilder::composite(const std::vector<ConvexHull>& hulls)
{
    if (hulls.empty()) return nullptr;

    // Count number of features to determine required mesh size
    uint32_t vertexCount = 0, faceCount = 0, indexCount = 0;

    for (const ConvexHull& hull : hulls) {
        vertexCount += hull.vertexCount();
        faceCount += hull.faces().faceCount();
        indexCount += hull.faces().indexCount();
    }

    auto mesh = std::make_shared<Mesh>(vertexCount, faceCount, indexCount);

    uint32_t vertexIdx = 0, *idxPtr = mesh->m_faces[0], *sizePtr = mesh->m_faces.faceSizes();
    const uint32_t *hPtr = nullptr;

    uint32_t idxOffset = 0;
    for (const ConvexHull& hull : hulls) {
        for (const glm::dvec3& vertex : hull.vertices()) mesh->m_vertices[vertexIdx++] = vertex;

        hPtr = hull.faces()[0];
        for (uint32_t i = 0; i < hull.faces().faceCount(); i++) {
            *sizePtr = hull.faces().faceSizes()[i];
            for (uint32_t j = 0; j < *sizePtr; j++) *idxPtr++ = *hPtr++ + idxOffset;
            sizePtr++;
        }

        idxOffset += hull.vertexCount();
    }

    mesh->initialize();

    return mesh;
}

//std::shared_ptr<Mesh> MeshBuilder::eliminateCoincidentVertices(const std::shared_ptr<Mesh>& mesh)
//{
//
//    // Convert vertex data to a more convenient format
//    std::vector<glm::dvec3> v(mesh->vertexCount());
//    for (uint32_t i = 0; i < v.size(); i++) v[i] = mesh->vertices()[i];
//
//    std::vector<std::vector<uint32_t>> f;
//    f.reserve(mesh->faceCount());
//
//    eliminateCoincidentVertices(mesh->faces(), v, f);
//
//    return std::make_shared<Mesh>(VertexArray(v), FaceArray(f));
//}

void MeshBuilder::eliminateCoincidentVertices(const FaceArray& srcFaces, std::vector<glm::dvec3>& vertices, std::vector<std::vector<uint32_t>>& faces)
{

    uint32_t count = 0;
    double tolerance = 1e-3, factor = 1 / tolerance; // TODO validate such high tolerance
    glm::dvec3 offset = 0.5 * glm::dvec3{1.0, 1.0, 1.0} * tolerance;
    std::unordered_map<size_t, uint32_t> vertexMap;
    std::vector<uint32_t> indexMap(vertices.size(), std::numeric_limits<uint32_t>::max());

    // Remove coincident vertices, recording indices to use to recover vertex mapping
    for (uint32_t i = 0; i < indexMap.size(); i++) {
        size_t h = hash(vertices[i - count] + offset, factor);

        auto it = vertexMap.find(h);
        if (it == vertexMap.end()) {
            indexMap[i] = vertexMap[h] = i - count;
        } else {
            vertices.erase(vertices.begin() + i - count);
            indexMap[i] = it->second;
            count++;
        }
    }

    // Recover proper vertex mapping, removing shared indices in each face (Arise due to consolodating coincident vertices)
    for (uint32_t i = 0; i < srcFaces.faceCount(); i++) {
        std::vector<uint32_t>& face = faces.emplace_back();
        auto facePtr = srcFaces[i];
        uint32_t last = indexMap[facePtr[srcFaces.faceSizes()[i] - 1]];

        for (uint32_t j = 0; j < srcFaces.faceSizes()[i]; j++) {
            if (last != indexMap[facePtr[j]]) {
                face.emplace_back(indexMap[facePtr[j]]);
            }

            last = indexMap[facePtr[j]];
        }

        if (face.size() < 3) faces.pop_back();
    }
}

//std::shared_ptr<Mesh> MeshBuilder::cleaned(const std::shared_ptr<Mesh>& mesh)
//{
//    return cleaned(mesh->vertices(), mesh->faces());
//}
//
//std::shared_ptr<Mesh> MeshBuilder::cleaned(const VertexArray& vertices, const FaceArray& faces)
//{
//
//    // Convert vertex data to a more convenient format
//    std::vector<glm::dvec3> v(vertices.length());
//    for (uint32_t i = 0; i < v.size(); i++) v[i] = vertices[i];
//
//    std::vector<std::vector<uint32_t>> f;
//    f.reserve(faces.faceCount());
//
//    eliminateCoincidentVertices(faces, v, f);
//
//    // Calculate face normals
//    std::vector<glm::dvec3> normals;
//    normals.reserve(f.size());
//    for (std::vector<uint32_t>& face : f) {
//        normals.push_back(glm::normalize(glm::cross(v[face[1]] - v[face[0]], v[face[2]] - v[face[0]])));
//    }
//
//    return cleaned(v, normals, FaceArray(f));
//}

size_t MeshBuilder::hash(const glm::dvec3& vec, double factor)
{
    return hash((uint32_t)(vec.x * factor), (uint32_t)(vec.y * factor), (uint32_t)(vec.z * factor));
}

size_t MeshBuilder::hash(size_t a, size_t b, size_t c)
{
//    std::cout << "> " << a << " " << b << " " << c << "\n";
    return cantor(a, cantor(b, c));
}

size_t MeshBuilder::cantor(size_t a, size_t b)
{
    return (a + b + 1) * (a + b) / 2 + b;
}

//std::shared_ptr<Mesh> MeshBuilder::cleaned(std::vector<glm::dvec3>& vertices, const std::vector<glm::dvec3>& normals, const FaceArray& faces)
//{
//
////    std::cout << "\033[31mMesh Input is: " << isManifold(faces) << "\033[0m\n";
//
//
//
//    // Generate linked list of faces and their neighbors
//    std::vector<std::vector<uint32_t>> neighbors = faces.adjacencies();
//    std::vector<std::vector<uint32_t>> indices;
//
//    std::cout << "XXXXXXXXXXXXXXXXXXXX\n";
////    for (const glm::dvec3& vertex : vertices) std::cout << vertex << "\n";
//    faces.print();
//
//    std::cout << "~~~~~~~~~~~~~~" << neighbors.size() << " " << faces.faceCount() << "~~~~~~~~~~~~~~\n";
//
//    faces.print();
//
//    // Identify faces to merge based on direction of the face normals
//    auto state = std::vector<uint8_t>(faces.faceCount());
//
//    // TODO handle more than just triangle primitives
//    for (uint32_t i = 0; i < faces.faceCount(); i++) {
//        if (state[i]) continue; // Skip faces that have already been considered
//        state[i] = 2;
//
//        std::vector<uint32_t> &face = indices.emplace_back();
//        auto ptr = faces[i];
//        for (uint32_t j = 0; j < faces.faceSizes()[i]; j++) face.emplace_back(ptr[j]);
//
////        std::cout << "F" << i << ": " << normals[i] << "\n|" << i << "| ";
//
//        for (unsigned int k : face) std::cout << k << " ";
//        std::cout << "| ";
//        for (uint32_t ni : neighbors[i]) std::cout << ni << " ";
//        std::cout << "<<<\n";
//
//        for (uint32_t j = 0; j < neighbors[i].size(); j++) {
////            std::cout << i << " " << j << " N" << neighbors[i][j] << "/" << (neighbors[i][j] == std::numeric_limits<uint32_t>::max() ? 0 : state[neighbors[i][j]]) << "\n";
//            if (neighbors[i][j] == std::numeric_limits<uint32_t>::max() || state[neighbors[i][j]]) continue;
//
//            uint32_t idx = neighbors[i][j], count = faces.faceSizes()[idx], reduc = 0;
//            if (count != 3) std::cout << "WARNING! Neighbors have more than 3 edges. Faces may not be cleaned properly\n";
//
////            std::cout << "DOT: " << normals[i].dot(normals[idx]) << "\n";
//            // std::numeric_limits<double>::epsilon() TODO identify reasonable tolerance
//            if (glm::dot(normals[i], normals[idx]) > 1 - 1e-12) { // If coplanar neighbors
//                ptr = faces[idx];
//
//                std::cout << "(" << i << " " << j << " " << idx << " " << 99 << ") ";
//                for (unsigned int k : face) std::cout << k << " ";
//                std::cout << "\n";
//
//                for (uint32_t k = 0; k < count; k++) std::cout << ptr[k] << " ";
//                std::cout << "\n";
//
//                uint32_t k = 0;
//                for (; k < count; k++) if (ptr[k] == face[j]) break;
//                std::cout << "-> " << k << "\n";
//
////                uint32_t vf = ptr[(k + 1) % count], vl = ptr[(k + count - 1) % count];
//////                std::cout << face[j + 1] << " " << face[(j + 2) % face.size()] << " " << v1 << " " << v2 << "||\n";
////
//
//                // Eliminate extras due to bridging
//                std::cout << "C" << face[(j + 2) % face.size()] << " " << ((-6) % 5) << "\n";
//                while (face[(j + 2) % face.size()] == ptr[(k + count - 2) % faces.faceSizes()[idx]]) {
//                    std::cout << "BRIDGING " << i << " " << j << " " << k << " " << face.size() << "\n";
//
//                    face.erase(face.begin() + j + 1);
//                    neighbors[i].erase(neighbors[i].begin() + j + 1);
//
//                    count--;
//                }
//
//                // Handle regular additions
//                for (uint32_t m = 1; m < count - 1; m++) {
//                    face.insert(face.begin() + j + m, ptr[(k + m) % faces.faceSizes()[idx]]);
//                }
//                // Insert neighbor contents to current face
//                neighbors[i][j] = neighbors[idx][k];
//                for (uint32_t m = 1; m < count - 1; m++) {
//                    neighbors[i].insert(neighbors[i].begin() + j + m, neighbors[idx][(k + m) % faces.faceSizes()[idx]]);
//                }
//
//                state[idx] = 1; // Prevent revisiting this face
//                j--; // Step back to avoid skipping new face link
//            }
//        }
//
//        // Temporary - Only resolve boundary edge == Ignores holes
//        auto ret = std::distance(std::find(face.rbegin(), face.rend(), face[0]), face.rend());
//        std::cout << "~~~~ " << std::distance(std::find(face.rbegin(), face.rend(), face[0]), face.rend()) << " " << std::distance(std::find(face.rbegin(), face.rend(), face[0]), face.rbegin()) << "\n";
//        if (ret != 1) {
//            face.erase(face.begin() + ret - 1, face.end());
//        }
//
//        // TODO Handle holes - Would need to split face into multiple separate faces to handle
//
//    }
//
//
//    std::cout << "Vertex Indices:\n";
//    for (const auto& loop : indices) {
//        for (auto l : loop) std::cout << l << " ";
//        std::cout << "\n";
//    }
//
//    // TODO re-include collinear culling
////    for (std::vector<uint32_t>& face: indices) {
////        for (uint32_t i = 0; i < face.size(); i++) {
////            uint32_t idx = (i + 1) % face.size();
////            if (dvec3::collinear(vertices[face[i]], vertices[face[idx]], vertices[face[(i + 2) % face.size()]])) {
////                face.erase(face.begin() + idx);
////                i--;
////            }
////        }
////    }
//
//    // Remove any strays
//    for (uint32_t i = 0; i < indices.size(); i++) {
//        if (indices[i].size() < 3) {
//            std::cout << "ERASING FACE" << i << "\n";
//            indices.erase(indices.begin() + i);
//        }
//    }
//
//    // Count instances of every vertex
//    std::vector<uint32_t> instances(vertices.size(), 0);
//    for (const std::vector<uint32_t>& face : indices) {
//        for (uint32_t vertex : face) instances[vertex]++;
//    }
//
//    // Determine appropriate vertex indexing with orphans removed
//    uint32_t idx = 0;
//    for (uint32_t& vertex : instances) vertex = vertex > 0 ? idx++ : std::numeric_limits<uint32_t>::max();
//
//    for (uint32_t& vertex : instances) std::cout << vertex << " ";
//    std::cout << "\nVV|" << vertices.size() << "|\n";
//
//    // Reorganize vertex list before removing orphans
//    for (uint32_t i = 0; i < instances.size(); i++) {
//        if (instances[i] != std::numeric_limits<uint32_t>::max()) vertices[instances[i]] = vertices[i];
//    }
//    vertices.erase(vertices.begin() + idx, vertices.end());
//
//    std::cout << "NV|" << vertices.size() << "|\n";
//
////     Correct vertex indexing of the faces
//    for (uint32_t i = 0; i < indices.size(); i++) {
//        for (uint32_t& vertex : indices[i]) {
//            std::cout << "|" << vertex << " " << instances[vertex] << "| ";
//            vertex = instances[vertex];
//        }
//
//        if (indices[i].size() < 3) {
//            indices.erase(indices.begin() + i);
//            i--;
//        }
//    }
//    std::cout << "\n";
//
//    std::cout << "!!!!!!!Vertex Indices:\n";
//    for (const auto& loop : indices) {
//        for (auto l : loop) std::cout << l << " ";
//        std::cout << "\n";
//    }
//
////    std::cout << "\033[31mMesh Output is: " << isManifold(FaceArray(indices)) << "\033[0m\n";
//
//
//    return std::make_shared<Mesh>(VertexArray(vertices), FaceArray(indices));
//}

bool MeshBuilder::isManifold(const std::shared_ptr<Mesh>& mesh)
{
    return isManifold(mesh->faces());
}
bool MeshBuilder::isManifold(const FaceArray& faces)
{
    const std::vector<std::vector<uint32_t>> neighbors = faces.adjacencies();

    for (const std::vector<uint32_t>& set : neighbors) {
        if (std::find(set.begin(), set.end(), std::numeric_limits<uint32_t>::max()) != set.end()) return false;
    }

    // TODO Requires checking for internal geometry and zero-thickness features for completeness

    return true;
}