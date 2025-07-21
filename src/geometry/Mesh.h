//
// Created by Cam on 2024-10-20.
//

#ifndef AUTOCARVER_MESH_H
#define AUTOCARVER_MESH_H

// Mesh manipulation

#include <cstdint>
#include <vector>

#include "fileIO/Serializable.h"

#include "MeshBuilder.h"
#include "geometry/primitives/ConvexHull.h"

#include "VertexArray.h"
#include "FaceArray.h"

static glm::vec3 NULL_COLOR = { 1.0f, 0.0f, 1.0f };


class Mesh : public Serializable {
public:

    friend class MeshBuilder;

    explicit Mesh(float vertices[], uint32_t vertexCount, uint32_t indices[], uint32_t indexCount);
    explicit Mesh(const ConvexHull& hull, bool applyColorPattern = true);

    explicit Mesh(const float *vertices, uint32_t vertexCount, const uint32_t *faceIndices, const uint32_t *faceSizes, uint32_t faceCount);
    explicit Mesh(const std::vector<glm::vec3>& vertices, const std::vector<Triangle>& faces);

    explicit Mesh(const std::string& filename);
    explicit Mesh(std::ifstream& file);

    explicit Mesh(VertexArray vertices, FaceArray faces);
    explicit Mesh(const Mesh& mesh);

    Mesh(uint32_t vertexCount, uint32_t faceCount, uint32_t indexCount);

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

    void scale(float scalar);
    void scale(const glm::vec3& scale);
    void translate(const glm::vec3& translation);
    void rotate(const glm::vec3& axis, float theta);

    void normalize(float scalar = 1.0f);
    void center();
    void zero();


    void xExtents(float &near, float &far) const;
    void yExtents(float &near, float &far) const;
    void zExtents(float &near, float &far) const;
    void extents(const glm::vec3& axis, float &near, float &far) const;

    float xSpan() const;
    float ySpan() const;
    float zSpan() const;

    void overrideColor(bool enable);
    void setBaseColor(const glm::vec3& color);

    void setVertexColor(const glm::vec3& color);
    void setVertexColor(uint32_t vertexIdx, const glm::vec3& color);

    void setFaceColor(const glm::vec3& color);
    void setFaceColor(uint32_t faceIdx, const glm::vec3& color);

    void calculateAdjacencies();

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const VertexArray& vertices() const;

    [[nodiscard]] const VertexArray& vertexNormals() const;
    [[nodiscard]] const std::vector<glm::vec3>& vertexColors() const;

    [[nodiscard]] const glm::vec3& baseColor() const;

    [[nodiscard]] bool colorsAssigned() const;
    [[nodiscard]] bool faceColorsAssigned() const;
    [[nodiscard]] bool vertexColorsAssigned() const;

    [[nodiscard]] bool useBaseColor() const;

    [[nodiscard]] uint32_t triangleCount() const;
    const uint32_t* indices() const;

    [[nodiscard]] uint32_t faceCount() const;
    [[nodiscard]] const FaceArray& faces() const;

    [[nodiscard]] uint32_t matchFace(const glm::vec3& axis);

    [[nodiscard]] std::vector<uint32_t> outline(const glm::vec3& axis);

    [[nodiscard]] float surfaceArea() const;

    [[nodiscard]] float volume() const;
    [[nodiscard]] glm::vec3 centroid() const;
    [[nodiscard]] glm::vec3 boundedOffset() const;

    [[nodiscard]] glm::mat3x3 inertiaTensor() const;

    std::vector<uint32_t> sharedFaces(const std::shared_ptr<Mesh>& reference) const;

    void print() const;

private:

    void initialize();

    void calculateVertexNormals();

    std::vector<uint16_t> identifyHorizonFaces(const glm::vec3& axis, const std::vector<std::vector<uint32_t>>& adjacencies) const;

    [[nodiscard]] float faceArea(uint32_t faceIdx) const;
    [[nodiscard]] static float faceArea(const std::vector<glm::vec3>& vertices) ;

    static float tetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
    static glm::mat3x3 tetrahedronInertiaTensor(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

private:

    VertexArray m_vertices;

    FaceArray m_faces;

    VertexArray m_vertexNormals;

    bool m_colorOverride;
    glm::vec3 m_baseColor;
    std::vector<glm::vec3> m_vertexColors;

    bool m_adjacencyOK;
    std::vector<std::vector<uint32_t>> m_adjacencies;

//    float *m_colors;

};

#endif //AUTOCARVER_MESH_H
