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
#include "fileIO/MeshHandler.h"
#include "geometry/primitives/ConvexHull.h"

#include "VertexArray.h"
#include "FaceArray.h"

class Ray;

static glm::dvec3 NULL_COLOR = { 1.0f, 0.0f, 1.0f };


class Mesh : public Serializable {
public:

    friend class MeshBuilder;
    friend class MeshHandler;

    explicit Mesh(const ConvexHull& hull, bool applyColorPattern = true);

    explicit Mesh(const std::vector<glm::dvec3>& vertices, const std::vector<TriIndex>& faces);

    explicit Mesh(const std::string& filename);
    explicit Mesh(std::ifstream& file);

    Mesh(uint32_t vertexCount, uint32_t faceCount, uint32_t indexCount);

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

    void scale(double scalar);
    void scale(const glm::dvec3& scale);
    void translate(const glm::dvec3& translation);
    void rotate(const glm::dvec3& axis, double theta);
    void rotate(const glm::dquat& rotation);
    void transform(const glm::dmat4& transform);

    void normalize(double scalar = 1.0f);
    void center();
    void zero();

    bool isInitialized() const;

    void xExtents(double &near, double &far) const;
    void yExtents(double &near, double &far) const;
    void zExtents(double &near, double &far) const;
    void extents(const glm::dvec3& axis, double &near, double &far) const;

    double xSpan() const;
    double ySpan() const;
    double zSpan() const;

    void overrideColor(bool enable);
    void setBaseColor(const glm::dvec3& color);

    void setVertexColor(const glm::dvec3& color);
    void setVertexColor(uint32_t vertexIdx, const glm::dvec3& color);

    void setFaceColor(const glm::dvec3& color);
    void setFaceColor(uint32_t faceIdx, const glm::dvec3& color);

    void calculateAdjacencies();

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const VertexArray& vertices() const;

    [[nodiscard]] const VertexArray& vertexNormals() const;
    [[nodiscard]] const std::vector<glm::dvec3>& vertexColors() const;

    [[nodiscard]] const glm::dvec3& baseColor() const;

    [[nodiscard]] bool colorsAssigned() const;
    [[nodiscard]] bool faceColorsAssigned() const;
    [[nodiscard]] bool vertexColorsAssigned() const;

    [[nodiscard]] bool useBaseColor() const;

    [[nodiscard]] uint32_t triangleCount() const;
    const uint32_t* indices() const;

    [[nodiscard]] uint32_t faceCount() const;
    [[nodiscard]] const FaceArray& faces() const;

    [[nodiscard]] uint32_t matchFace(const glm::dvec3& axis);

    [[nodiscard]] std::vector<uint32_t> outline(const glm::dvec3& axis);

    [[nodiscard]] double surfaceArea() const;

    [[nodiscard]] double volume() const;
    [[nodiscard]] glm::dvec3 centroid() const;
    [[nodiscard]] glm::dvec3 boundedOffset() const;

    [[nodiscard]] glm::dmat3 inertiaTensor() const;

    std::vector<uint32_t> sharedFaces(const std::shared_ptr<Mesh>& reference) const;

    std::tuple<bool, double> raycast(const Ray& ray) const;
    std::tuple<bool, double, uint32_t> pickFace(const Ray& ray) const;

    void print() const;

private:

    Mesh();

    void initialize();

    void calculateVertexNormals();

    std::vector<uint16_t> identifyHorizonFaces(const glm::dvec3& axis, const std::vector<std::vector<uint32_t>>& adjacencies) const;

    [[nodiscard]] double faceArea(uint32_t faceIdx) const;
    [[nodiscard]] static double faceArea(const std::vector<glm::dvec3>& vertices) ;

    static double tetrahedronVolume(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c);
    static glm::mat3x3 tetrahedronInertiaTensor(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c);

private:

    bool m_initialized;

    VertexArray m_vertices;
    FaceArray m_faces;

    VertexArray m_vertexNormals;

    bool m_colorOverride;
    glm::dvec3 m_baseColor;
    std::vector<glm::dvec3> m_vertexColors;

    bool m_adjacencyOK;
    std::vector<std::vector<uint32_t>> m_adjacencies;

//    double *m_colors;

};

#endif //AUTOCARVER_MESH_H
