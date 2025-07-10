//
// Created by Cam on 2024-10-25.
//

#include "MeshBuilder.h"

#include "Octree.h"

#include <glm/glm.hpp>
#include <array>
#include <unordered_map>

std::shared_ptr<Mesh> MeshBuilder::plane(float width, const glm::vec3& origin, const glm::vec3& normal)
{
    glm::vec3 ref = normal.y == 1 ? glm::vec3{1.0f, 0.0f, 0.0f} : glm::vec3{0.0f, 1.0f, 0.0f};
    return plane(width, width, origin, normal, ref);
}

std::shared_ptr<Mesh> MeshBuilder::plane(float length, float width, const glm::vec3& origin, const glm::vec3& normal, const glm::vec3& ref)
{

    glm::vec3 wAxis = glm::normalize(glm::cross(normal, ref)), lAxis = glm::normalize(glm::cross(normal, wAxis));
    glm::vec3 a = origin + wAxis * width * 0.5f + lAxis * length * 0.5f, b = a - wAxis * width, c = b - lAxis * length, d = c + wAxis * width;

    auto vertices = VertexArray(4);
    vertices[0] = { a.x, a.y, a.z };
    vertices[1] = { b.x, b.y, c.z };
    vertices[2] = { c.x, c.y, b.z };
    vertices[3] = { d.x, d.y, d.z };

    auto faces = FaceArray(2, 8);
    uint32_t *idxPtr = faces[0], *sizePtr = faces.faceSizes();

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

    return std::make_shared<Mesh>(vertices, faces);
}

std::shared_ptr<Mesh> MeshBuilder::box(float sideLength)
{
    return box(sideLength, sideLength, sideLength);
}

std::shared_ptr<Mesh> MeshBuilder::box(float length, float width, float height)
{
    length /= 2;
    width /=2;
    height /=2;

    auto vertices = VertexArray(8);
    vertices[0] = { -length, -height, -width };
    vertices[1] = { -length,  height, -width };
    vertices[2] = {  length,  height, -width };
    vertices[3] = {  length, -height, -width };
    vertices[4] = { -length, -height,  width };
    vertices[5] = { -length,  height,  width };
    vertices[6] = {  length,  height,  width };
    vertices[7] = {  length, -height,  width };

    auto faces = FaceArray(6, 24);
    uint32_t *idxPtr = faces[0], *sizePtr = faces.faceSizes();
    indexBox(idxPtr, sizePtr);

    return std::make_shared<Mesh>(vertices, faces);
}

std::shared_ptr<Mesh> MeshBuilder::cylinder(float radius, float height, uint32_t segments){
    uint32_t vertexCount = 2 * segments, faceCount = 2 + segments;

    // Generate vertices to approximate a cylinder
    auto *vertices = new float[3 * vertexCount], *vPtr = vertices;

    float theta = 0;
    float inc = 2 * M_PI / segments;
    while(theta < 2 * M_PI - 1e-6){
        *vPtr++ = radius * cosf(theta);
        *vPtr++ = 0;
        *vPtr++ = radius * sinf(theta);

        *vPtr++ = radius * cosf(theta);
        *vPtr++ = height;
        *vPtr++ = radius * sinf(theta);

        theta += inc;
    }

    auto *faces = new uint32_t[2 * vertexCount + 4 * segments], *fPtr = faces;
    auto *faceSizes = new uint32_t[faceCount], *fsPtr = faceSizes;

    // Generate flat faces
    for(uint32_t i = 0; i < vertexCount; i+=2) *fPtr++ = i;
    for(uint32_t i = 1; i < vertexCount; i+=2) *fPtr++ = vertexCount - i;

    *fsPtr++ = vertexCount / 2;
    *fsPtr++ = vertexCount / 2;

    // Generate cylindrical faces
    for (uint32_t i = 0; i < segments; i++) {
        *fPtr++ = 2 * i;
        *fPtr++ = 2 * i + 1;
        *fPtr++ = (2 * i + 3) % (2 * segments);
        *fPtr++ = (2 * i + 2) % (2 * segments);
        *fsPtr++ = 4;
    }

    return std::make_shared<Mesh>(vertices, vertexCount, faces, faceSizes, faceCount);

}

std::shared_ptr<Mesh> MeshBuilder::extrude(const std::vector<glm::vec3>& border, const glm::vec3& normal, float depth){
    if (border.size() < 3) return nullptr;

    uint32_t vertexCount = 2 * border.size(), segments = border.size(), faceCount = 2 + segments;

    // Generate vertices to approximate a cylinder
    auto *vertices = new float[3 * vertexCount], *vPtr = vertices;

    for (const glm::vec3& vertex : border) {
        *vPtr++ = vertex.x;
        *vPtr++ = vertex.y;
        *vPtr++ = vertex.z;

        *vPtr++ = vertex.x + normal.x * depth;
        *vPtr++ = vertex.y + normal.y * depth;
        *vPtr++ = vertex.z + normal.z * depth;
    }

    auto *faces = new uint32_t[2 * vertexCount + 4 * segments], *fPtr = faces;
    auto *faceSizes = new uint32_t[faceCount], *fsPtr = faceSizes;

    // Generate flat faces
    for(uint32_t i = 0; i < vertexCount; i+=2) *fPtr++ = i;
    for(uint32_t i = 1; i < vertexCount; i+=2) *fPtr++ = vertexCount - i;

    *fsPtr++ = vertexCount / 2;
    *fsPtr++ = vertexCount / 2;

    // Generate border faces
    for (uint32_t i = 0; i < segments; i++) {
        *fPtr++ = 2 * i;
        *fPtr++ = 2 * i + 1;
        *fPtr++ = (2 * i + 3) % (2 * segments);
        *fPtr++ = (2 * i + 2) % (2 * segments);
        *fsPtr++ = 4;
    }

    return std::make_shared<Mesh>(vertices, vertexCount, faces, faceSizes, faceCount);
}

std::shared_ptr<Mesh> MeshBuilder::icosahedron(float radius){
    std::vector<glm::vec3> vertices;
    std::vector<Triangle> faces;

    icosahedron(radius, vertices, faces);

    return std::make_shared<Mesh>(vertices, faces);
}

void MeshBuilder::icosahedron(float radius, std::vector<glm::vec3>& vertices, std::vector<Triangle>& faces)
{

    vertices.reserve(12);
    faces.reserve(20);

    // Prepare icosahedron vertices
    auto t = (float)(1 + sqrt(5) / 2);

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
    for (glm::vec3& vertex : vertices) vertex = radius * glm::normalize(vertex);

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


std::shared_ptr<Mesh> MeshBuilder::icosphere(float radius, uint8_t subdivisions)
{
    uint32_t vertexCount = 10 * (uint32_t)pow(2, 2 * subdivisions) + 2;

    std::vector<glm::vec3> vertices;
    vertices.reserve(vertexCount);

    std::vector<Triangle> faces;
//    faces.reserve(20 * 4 * subdivisions);

    icosahedron(radius, vertices, faces);


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
            faces[j] = Triangle{a, b, c};
        }
    }


    return std::make_shared<Mesh>(vertices, faces);
}

uint32_t MeshBuilder::getMidPoint(std::vector<glm::vec3>& vertices, std::map<uint64_t, uint32_t> &table, uint64_t iA, uint64_t iB, float scalar){

    uint64_t key = iA > iB ? (iA << 32) + iB : (iB << 32) + iA;

    auto it = table.find(key);
    if(it == table.end()){
        vertices.push_back(scalar * glm::normalize(vertices[iA] + vertices[iB]));
        table[key] = vertices.size() - 1;
        return vertices.size() - 1;
    }

    return table[key];
}

std::shared_ptr<Mesh> MeshBuilder::mesh(const std::shared_ptr<Octree>& tree)
{
    uint32_t count = tree->octantCount(), idx = 0;
    std::cout << count << " / " << tree->maximumOctantCount() << " COUNTS\n";

    struct Item {
        const Octant* octant;
        const glm::vec3 offset;
        const float length;
    };

    auto vertices = VertexArray(8 * count);

    auto faces = FaceArray(6 * count, 24 * count);
    uint32_t *idxPtr = faces[0], *sizePtr = faces.faceSizes();

    std::vector<Item> items = { { tree->root(), tree->top(), tree->length() } };
    while (!items.empty()) {
        Item item = items[items.size() - 1];
        items.pop_back();

        if (item.octant->status == 2) continue;

        if (item.octant->status == 0) {
            uint32_t idxOffset = 8 * idx++;

            vertices[idxOffset    ] = item.offset;
            vertices[idxOffset + 1] = item.offset + glm::vec3{           0, item.length,           0 };
            vertices[idxOffset + 2] = item.offset + glm::vec3{ item.length, item.length,           0 };
            vertices[idxOffset + 3] = item.offset + glm::vec3{ item.length,           0,           0 };
            vertices[idxOffset + 4] = item.offset + glm::vec3{           0,           0, item.length };
            vertices[idxOffset + 5] = item.offset + glm::vec3{           0, item.length, item.length };
            vertices[idxOffset + 6] = item.offset + glm::vec3{ item.length, item.length, item.length };
            vertices[idxOffset + 7] = item.offset + glm::vec3{ item.length,           0, item.length };

            indexBox(idxPtr, sizePtr, idxOffset);
            idxPtr += 24;
            sizePtr += 6;
        }

        float length = 0.5f * item.length;
        for (uint32_t i = 0; i < 8; i++) {
            glm::vec3 offset = tree->octantOffset(i, length) + item.offset;

            if (item.octant->children[i] != nullptr) {
                items.push_back(Item{ item.octant->children[i], offset, length });
            }
        }
    }

    return std::make_shared<Mesh>(vertices, faces);
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
    // Some values directly from the meshes
    uint32_t aVC = a->vertexCount(), bVC = b->vertexCount();
    uint32_t aFC = a->faceCount(), bFC = b->faceCount();
    uint32_t aIC = a->faces().indexCount(), bIC = b->faces().indexCount();

    // Copy vertex data directly
    uint32_t vertexCount = aVC + bVC;
    auto *vertices = new float[3 * vertexCount], *vPtr = vertices;

    const float *aVPtr = (float*)a->vertices().vertices().data(), *bVPtr = (float*)b->vertices().vertices().data();
    for (uint32_t i = 0; i < 3 * aVC; i++) *vPtr++ = *aVPtr++;
    for (uint32_t i = 0; i < 3 * bVC; i++) *vPtr++ = *bVPtr++;


    // Count the number of face indices required
    uint32_t faceCount = aFC + bFC, indexCount = aIC + bIC;
    auto *faceSizes = new uint32_t[faceCount], *fsPtr = faceSizes;

    const uint32_t *aFSPtr = a->faces().faceSizes(), *bFSPtr = b->faces().faceSizes();
    for (uint32_t i = 0; i < aFC; i++) *fsPtr++ = *aFSPtr++;
    for (uint32_t i = 0; i < bFC; i++) *fsPtr++ = *bFSPtr++;


    // Copy face data
    auto *faces = new uint32_t[indexCount], *fPtr = faces;
    const uint32_t *aPtr = a->faces().faces(), *bPtr = b->faces().faces();
    for (uint32_t i = 0; i < aIC; i++) *fPtr++ = *aPtr++;
    for (uint32_t i = 0; i < bIC; i++) *fPtr++ = aVC + *bPtr++;


    // Create the merged mesh
    auto mesh = std::make_shared<Mesh>(vertices, vertexCount, faces, faceSizes, faceCount);

    // Copy colors
    mesh->setBaseColor(a->baseColor());
    if (a->faceColorsAssigned())
        for (uint32_t i = 0; i < aFC; i++) mesh->setFaceColor(i, a->faces().color(i));
    else mesh->setFaceColor(a->baseColor());

    if (b->faceColorsAssigned())
        for (uint32_t i = 0; i < bFC; i++) mesh->setFaceColor(aFC + i, b->faces().color(i));
    else
        for (uint32_t i = 0; i < bFC; i++) mesh->setFaceColor(aFC + i, b->baseColor());

    return mesh;
}

std::shared_ptr<Mesh> MeshBuilder::composite(const std::vector<ConvexHull>& hulls)
{
    uint32_t vertexCount = 0, faceCount = 0, indexCount = 0;

    for (const ConvexHull& hull : hulls) {
        std::cout << hull.vertexCount() << " " << hull.faces().faceCount() << " " << hull.faces().indexCount() << "~~\n";

        vertexCount += hull.vertexCount();
        faceCount += hull.faces().faceCount();
        indexCount += hull.faces().indexCount();
    }

    auto vertices = VertexArray(vertexCount);

    auto faces = FaceArray(faceCount, indexCount);
    uint32_t vertexIdx = 0, *sizePtr = faces.faceSizes(), *idxPtr = faces[0];
    const uint32_t *hPtr = nullptr;

    uint32_t idxOffset = 0;
    for (const ConvexHull& hull : hulls) {
        for (const glm::vec3& vertex : hull.vertices()) vertices[vertexIdx++] = vertex;

        hPtr = hull.faces()[0];
        for (uint32_t i = 0; i < hull.faces().faceCount(); i++) {
            *sizePtr = hull.faces().faceSizes()[i];
            for (uint32_t j = 0; j < *sizePtr; j++) *idxPtr++ = *hPtr++ + idxOffset;
            sizePtr++;
        }

        idxOffset += hull.vertexCount();
    }

    return std::make_shared<Mesh>(vertices, faces);
}

std::shared_ptr<Mesh> MeshBuilder::eliminateCoincidentVertices(const std::shared_ptr<Mesh>& mesh)
{

    // Convert vertex data to a more convenient format
    std::vector<glm::vec3> v(mesh->vertexCount());
    for (uint32_t i = 0; i < v.size(); i++) v[i] = mesh->vertices()[i];

    std::vector<std::vector<uint32_t>> f;
    f.reserve(mesh->faceCount());

    eliminateCoincidentVertices(mesh->faces(), v, f);

    return std::make_shared<Mesh>(VertexArray(v), FaceArray(f));
}

void MeshBuilder::eliminateCoincidentVertices(const FaceArray& srcFaces, std::vector<glm::vec3>& vertices, std::vector<std::vector<uint32_t>>& faces)
{

    uint32_t count = 0;
    float tolerance = 1e-3, factor = 1 / tolerance; // TODO validate such high tolerance
    glm::vec3 offset = 0.5f * glm::vec3{1.0f, 1.0f, 1.0f} * tolerance;
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

std::shared_ptr<Mesh> MeshBuilder::cleaned(const std::shared_ptr<Mesh>& mesh)
{
    return cleaned(mesh->vertices(), mesh->faces());
}

std::shared_ptr<Mesh> MeshBuilder::cleaned(const VertexArray& vertices, const FaceArray& faces)
{

    // Convert vertex data to a more convenient format
    std::vector<glm::vec3> v(vertices.length());
    for (uint32_t i = 0; i < v.size(); i++) v[i] = vertices[i];

    std::vector<std::vector<uint32_t>> f;
    f.reserve(faces.faceCount());

    eliminateCoincidentVertices(faces, v, f);

    // Calculate face normals
    std::vector<glm::vec3> normals;
    normals.reserve(f.size());
    for (std::vector<uint32_t>& face : f) {
        normals.push_back(glm::normalize(glm::cross(v[face[1]] - v[face[0]], v[face[2]] - v[face[0]])));
    }

    return cleaned(v, normals, FaceArray(f));
}

size_t MeshBuilder::hash(const glm::vec3& vec, float factor)
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

std::shared_ptr<Mesh> MeshBuilder::cleaned(std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals, const FaceArray& faces)
{

//    std::cout << "\033[31mMesh Input is: " << isManifold(faces) << "\033[0m\n";



    // Generate linked list of faces and their neighbors
    std::vector<std::vector<uint32_t>> neighbors = faces.adjacencies();
    std::vector<std::vector<uint32_t>> indices;

    std::cout << "XXXXXXXXXXXXXXXXXXXX\n";
//    for (const glm::vec3& vertex : vertices) std::cout << vertex << "\n";
    faces.print();

    std::cout << "~~~~~~~~~~~~~~" << neighbors.size() << " " << faces.faceCount() << "~~~~~~~~~~~~~~\n";

    faces.print();

    // Identify faces to merge based on direction of the face normals
    auto state = std::vector<uint8_t>(faces.faceCount());

    // TODO handle more than just triangle primitives
    for (uint32_t i = 0; i < faces.faceCount(); i++) {
        if (state[i]) continue; // Skip faces that have already been considered
        state[i] = 2;

        std::vector<uint32_t> &face = indices.emplace_back();
        auto ptr = faces[i];
        for (uint32_t j = 0; j < faces.faceSizes()[i]; j++) face.emplace_back(ptr[j]);

//        std::cout << "F" << i << ": " << normals[i] << "\n|" << i << "| ";

        for (unsigned int k : face) std::cout << k << " ";
        std::cout << "| ";
        for (uint32_t ni : neighbors[i]) std::cout << ni << " ";
        std::cout << "<<<\n";

        for (uint32_t j = 0; j < neighbors[i].size(); j++) {
//            std::cout << i << " " << j << " N" << neighbors[i][j] << "/" << (neighbors[i][j] == std::numeric_limits<uint32_t>::max() ? 0 : state[neighbors[i][j]]) << "\n";
            if (neighbors[i][j] == std::numeric_limits<uint32_t>::max() || state[neighbors[i][j]]) continue;

            uint32_t idx = neighbors[i][j], count = faces.faceSizes()[idx], reduc = 0;
            if (count != 3) std::cout << "WARNING! Neighbors have more than 3 edges. Faces may not be cleaned properly\n";

//            std::cout << "DOT: " << normals[i].dot(normals[idx]) << "\n";
            // std::numeric_limits<float>::epsilon() TODO identify reasonable tolerance
            if (glm::dot(normals[i], normals[idx]) > 1 - 1e-6) { // If coplanar neighbors
                ptr = faces[idx];

                std::cout << "(" << i << " " << j << " " << idx << " " << 99 << ") ";
                for (unsigned int k : face) std::cout << k << " ";
                std::cout << "\n";

                for (uint32_t k = 0; k < count; k++) std::cout << ptr[k] << " ";
                std::cout << "\n";

                uint32_t k = 0;
                for (; k < count; k++) if (ptr[k] == face[j]) break;
                std::cout << "-> " << k << "\n";

//                uint32_t vf = ptr[(k + 1) % count], vl = ptr[(k + count - 1) % count];
////                std::cout << face[j + 1] << " " << face[(j + 2) % face.size()] << " " << v1 << " " << v2 << "||\n";
//

                // Eliminate extras due to bridging
                std::cout << "C" << face[(j + 2) % face.size()] << " " << ((-6) % 5) << "\n";
                while (face[(j + 2) % face.size()] == ptr[(k + count - 2) % faces.faceSizes()[idx]]) {
                    std::cout << "BRIDGING " << i << " " << j << " " << k << " " << face.size() << "\n";

                    face.erase(face.begin() + j + 1);
                    neighbors[i].erase(neighbors[i].begin() + j + 1);

                    count--;
                }

                // Handle regular additions
                for (uint32_t m = 1; m < count - 1; m++) {
                    face.insert(face.begin() + j + m, ptr[(k + m) % faces.faceSizes()[idx]]);
                }
                // Insert neighbor contents to current face
                neighbors[i][j] = neighbors[idx][k];
                for (uint32_t m = 1; m < count - 1; m++) {
                    neighbors[i].insert(neighbors[i].begin() + j + m, neighbors[idx][(k + m) % faces.faceSizes()[idx]]);
                }

                state[idx] = 1; // Prevent revisiting this face
                j--; // Step back to avoid skipping new face link
            }
        }

        // Temporary - Only resolve boundary edge == Ignores holes
        auto ret = std::distance(std::find(face.rbegin(), face.rend(), face[0]), face.rend());
        std::cout << "~~~~ " << std::distance(std::find(face.rbegin(), face.rend(), face[0]), face.rend()) << " " << std::distance(std::find(face.rbegin(), face.rend(), face[0]), face.rbegin()) << "\n";
        if (ret != 1) {
            face.erase(face.begin() + ret - 1, face.end());
        }

        // TODO Handle holes - Would need to split face into multiple separate faces to handle

    }


    std::cout << "Vertex Indices:\n";
    for (const auto& loop : indices) {
        for (auto l : loop) std::cout << l << " ";
        std::cout << "\n";
    }

    // TODO re-include collinear culling
//    for (std::vector<uint32_t>& face: indices) {
//        for (uint32_t i = 0; i < face.size(); i++) {
//            uint32_t idx = (i + 1) % face.size();
//            if (vec3f::collinear(vertices[face[i]], vertices[face[idx]], vertices[face[(i + 2) % face.size()]])) {
//                face.erase(face.begin() + idx);
//                i--;
//            }
//        }
//    }

    // Remove any strays
    for (uint32_t i = 0; i < indices.size(); i++) {
        if (indices[i].size() < 3) {
            std::cout << "ERASING FACE" << i << "\n";
            indices.erase(indices.begin() + i);
        }
    }

    // Count instances of every vertex
    std::vector<uint32_t> instances(vertices.size(), 0);
    for (const std::vector<uint32_t>& face : indices) {
        for (uint32_t vertex : face) instances[vertex]++;
    }

    // Determine appropriate vertex indexing with orphans removed
    uint32_t idx = 0;
    for (uint32_t& vertex : instances) vertex = vertex > 0 ? idx++ : std::numeric_limits<uint32_t>::max();

    for (uint32_t& vertex : instances) std::cout << vertex << " ";
    std::cout << "\nVV|" << vertices.size() << "|\n";

    // Reorganize vertex list before removing orphans
    for (uint32_t i = 0; i < instances.size(); i++) {
        if (instances[i] != std::numeric_limits<uint32_t>::max()) vertices[instances[i]] = vertices[i];
    }
    vertices.erase(vertices.begin() + idx, vertices.end());

    std::cout << "NV|" << vertices.size() << "|\n";

//     Correct vertex indexing of the faces
    for (uint32_t i = 0; i < indices.size(); i++) {
        for (uint32_t& vertex : indices[i]) {
            std::cout << "|" << vertex << " " << instances[vertex] << "| ";
            vertex = instances[vertex];
        }

        if (indices[i].size() < 3) {
            indices.erase(indices.begin() + i);
            i--;
        }
    }
    std::cout << "\n";

    std::cout << "!!!!!!!Vertex Indices:\n";
    for (const auto& loop : indices) {
        for (auto l : loop) std::cout << l << " ";
        std::cout << "\n";
    }

//    std::cout << "\033[31mMesh Output is: " << isManifold(FaceArray(indices)) << "\033[0m\n";


    return std::make_shared<Mesh>(VertexArray(vertices), FaceArray(indices));
}

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