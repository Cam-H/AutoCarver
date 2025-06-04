//
// Created by Cam on 2024-10-20.
//

#ifndef AUTOCARVER_MESH_H
#define AUTOCARVER_MESH_H

// Mesh manipulation

#include <cstdint>
#include <vector>

#include "fileIO/Serializable.h"

#include "ConvexHull.h"

#include "VertexArray.h"
#include "FaceArray.h"

static glm::vec3 NULL_COLOR = { 1.0f, 0.0f, 1.0f };


class Mesh : public Serializable {
public:

    explicit Mesh(float vertices[], uint32_t vertexCount, uint32_t indices[], uint32_t indexCount);
    explicit Mesh(const ConvexHull& hull, bool applyColorPattern = true);

    explicit Mesh(const float *vertices, uint32_t vertexCount, const uint32_t *faceIndices, const uint32_t *faceSizes, uint32_t faceCount);
    explicit Mesh(const std::vector<glm::vec3>& vertices, const std::vector<Triangle>& faces);

    explicit Mesh(const std::string& filename);
    explicit Mesh(std::ifstream& file);

    explicit Mesh(VertexArray vertices, FaceArray faces);

    ~Mesh();

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

    void scale(float scalar);
    void scale(float x, float y, float z);
    void translate(float x, float y, float z);
    void rotate(float x, float y, float z, float theta);

    void zero();


    void xExtents(float &near, float &far) const;
    void yExtents(float &near, float &far) const;
    void zExtents(float &near, float &far) const;
    void extents(const glm::vec3& axis, float &near, float &far) const;

    void setBaseColor(const glm::vec3& color);
    void setFaceColor(uint32_t faceIdx, const glm::vec3& color);

    void applyColorOverride(const glm::vec3& color);
    void setColorOverride(const glm::vec3& color);
    void enableColorOverride(bool enable = true);
    void disableColorOverride();

    void calculateAdjacencies();

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const VertexArray& vertices() const;

    [[nodiscard]] const VertexArray& faceNormals() const;
    [[nodiscard]] const VertexArray& vertexNormals() const;

    [[nodiscard]] const VertexArray& colors() const;
    [[nodiscard]] const glm::vec3& baseColor() const;
    [[nodiscard]] glm::vec3 faceColor(uint32_t faceIdx) const;
    [[nodiscard]] const glm::vec3& colorOverride() const;

    [[nodiscard]] bool faceColorsAssigned() const;
    [[nodiscard]] bool colorOverrideEnabled() const;

    [[nodiscard]] uint32_t triangleCount() const;
    uint32_t* indices();

    [[nodiscard]] uint32_t faceCount() const;
    [[nodiscard]] const FaceArray& faces() const;

    [[nodiscard]] std::vector<uint32_t> outline(const glm::vec3& axis);

    [[nodiscard]] float surfaceArea() const;

    [[nodiscard]] float volume() const;
    [[nodiscard]] glm::vec3 centroid() const;
    [[nodiscard]] glm::mat3x3 inertiaTensor() const;

    std::vector<uint32_t> sharedFaces(const std::shared_ptr<Mesh>& reference) const;

    void directRepresentation(float *vertices, float *normals, float *colors);

    void print() const;

private:

    explicit Mesh();

    void initialize(bool prepareIndexing = true);

    void calculateFaceNormals();
    void calculateVertexNormals();

    std::vector<uint16_t> identifyHorizonFaces(const glm::vec3& axis, const std::vector<std::vector<uint32_t>>& adjacencies) const;

    [[nodiscard]] float faceArea(uint32_t faceIdx) const;
    [[nodiscard]] static float faceArea(const std::vector<glm::vec3>& vertices) ;

    static float tetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
    static glm::mat3x3 tetrahedronInertiaTensor(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

private:

    VertexArray m_vertices;

    FaceArray m_faces;

    VertexArray m_faceNormals;
    VertexArray m_vertexNormals;

    uint32_t *m_indices;
    uint32_t m_indexCount;

    VertexArray m_colors;
    glm::vec3 m_baseColor;
    glm::vec3 m_colorOverride;
    bool m_colorOverrideEnable;

    bool m_adjacencyOK;
    std::vector<std::vector<uint32_t>> m_adjacencies;

//    float *m_colors;

    const static uint8_t STRIDE = 3;

};

#endif //AUTOCARVER_MESH_H
