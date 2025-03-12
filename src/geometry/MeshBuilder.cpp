//
// Created by Cam on 2024-10-25.
//

#include "MeshBuilder.h"

#include <array>
#include <unordered_map>

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

std::shared_ptr<Mesh> MeshBuilder::eliminateCoincidentVertices(const std::shared_ptr<Mesh>& mesh)
{

    // Convert vertex data to a more convenient format
    std::vector<vec3f> v(mesh->vertexCount());
    for (uint32_t i = 0; i < v.size(); i++) v[i] = mesh->vertices()[i];

    std::vector<std::vector<uint32_t>> f;
    f.reserve(mesh->faceCount());

    eliminateCoincidentVertices(mesh->faces(), v, f);

    return std::make_shared<Mesh>(VertexArray(v), FaceArray(f));
}

void MeshBuilder::eliminateCoincidentVertices(const FaceArray& srcFaces, std::vector<vec3f>& vertices, std::vector<std::vector<uint32_t>>& faces)
{

    uint32_t count = 0;
    float tolerance = 1e-3, factor = 1 / tolerance; // TODO validate such high tolerance
    vec3f offset = 0.5f * vec3f{1, 1, 1} * tolerance;
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
    std::vector<vec3f> v(vertices.length());
    for (uint32_t i = 0; i < v.size(); i++) v[i] = vertices[i];

    std::vector<std::vector<uint32_t>> f;
    f.reserve(faces.faceCount());

    eliminateCoincidentVertices(faces, v, f);

    // Calculate face normals
    std::vector<vec3f> normals;
    normals.reserve(f.size());
    for (std::vector<uint32_t>& face : f) {
        normals.push_back(vec3f::unitNormal(v[face[0]], v[face[1]], v[face[2]]));
    }

    return cleaned(v, normals, FaceArray(f));
}

size_t MeshBuilder::hash(const vec3f& vec, float factor)
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

std::shared_ptr<Mesh> MeshBuilder::cleaned(std::vector<vec3f>& vertices, const std::vector<vec3f>& normals, const FaceArray& faces)
{

//    std::cout << "\033[31mMesh Input is: " << isManifold(faces) << "\033[0m\n";



    // Generate linked list of faces and their neighbors
    std::vector<std::vector<uint32_t>> neighbors = faces.adjacencies();
    std::vector<std::vector<uint32_t>> indices;

    std::cout << "XXXXXXXXXXXXXXXXXXXX\n";
    for (const vec3f& vertex : vertices) std::cout << vertex << "\n";
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

        std::cout << "F" << i << ": " << normals[i] << "\n|" << i << "| ";

        for (unsigned int k : face) std::cout << k << " ";
        std::cout << "| ";
        for (uint32_t ni : neighbors[i]) std::cout << ni << " ";
        std::cout << "<<<\n";

        for (uint32_t j = 0; j < neighbors[i].size(); j++) {
//            std::cout << i << " " << j << " N" << neighbors[i][j] << "/" << (neighbors[i][j] == std::numeric_limits<uint32_t>::max() ? 0 : state[neighbors[i][j]]) << "\n";
            if (neighbors[i][j] == std::numeric_limits<uint32_t>::max() || state[neighbors[i][j]]) continue;

            uint32_t idx = neighbors[i][j], count = faces.faceSizes()[idx], reduc = 0;
            if (count != 3) std::cout << "WARNING! Neighbors have more than 3 edges. Faces may not be cleaned properly\n";

            std::cout << "DOT: " << normals[i].dot(normals[idx]) << "\n";
            // std::numeric_limits<float>::epsilon() TODO identify reasonable tolerance
            if (normals[i].dot(normals[idx]) > 1 - 1e-6) { // If coplanar neighbors
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

    for (std::vector<uint32_t>& face: indices) {
        for (uint32_t i = 0; i < face.size(); i++) {
            uint32_t idx = (i + 1) % face.size();
            if (vec3f::collinear(vertices[face[i]], vertices[face[idx]], vertices[face[(i + 2) % face.size()]])) {
                face.erase(face.begin() + idx);
                i--;
            }
        }
    }

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