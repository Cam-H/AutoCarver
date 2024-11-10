//
// Created by cameronh on 24/08/23.
//

#ifndef AUTOCARVER_CONVEXHULL_H
#define AUTOCARVER_CONVEXHULL_H

#include <QColor>
//#include <glm/glm.hpp>

#include <string>
#include <memory>
#include <vector>

//#include "Sphere.h"
//#include "Face.h"
//#include "VHACD.h"

#include "VertexArray.h"

//const static QColor DEFAULT_CONVEX_HULL_COLOR = {166, 87, 51};
//
//const static std::vector<glm::vec3> COLOR_CYCLE = {
//        {-0.12f, 0.56f, -0.72f},
//        {1.00f, 0.51f, -0.06f},
//        {-0.14f,  0.62f, -0.14f},
//        {-0.91f, 0.11f,  -0.12f},
//        {-0.53f, -0.41f, -0.92f},
//        {0.48f, -0.38f,  0.25f},
//        {-0.89f, -0.44f, 0.93f},
//        {0.50f, 0.50f, -0.50f},
//        {0.00f, -0.10f, 0.00f},
//        {-0.33f, -0.33f, -0.33f},
//        {0.33f, -1.00f, 1.00f}
//};
//

class ConvexHull {
public:

    ConvexHull();
    ConvexHull(const float* cloud, uint32_t cloudSize);

    ConvexHull(const ConvexHull& rhs) = default;
//    ConvexHull& operator=(const ConvexHull& rhs){};

    void scale(float distance);


    uint32_t vertexCount() const;
    float* vertices() const;

//    uint32_t *facets() const;
    uint32_t *facetSizes() const;
    uint32_t facetCount() const;

    uint32_t* triangulate(uint32_t& triangleCount) const;

//    [[nodiscard]] const std::vector<glm::vec3>& getVertices() const;
//    [[nodiscard]] const std::vector<glm::vec3>& getNormals() const;
//    [[nodiscard]] const std::vector<Triangle>& getTriangles() const;

//    [[nodiscard]] std::vector<Face> getFaces() const;

//    [[nodiscard]] std::vector<std::pair<glm::vec3, glm::vec3>> getAxes() const;


//    [[nodiscard]] bool isSourceConvex() const;
//    [[nodiscard]] float getVolume() const;
//    [[nodiscard]] const Sphere& getBoundingSphere() const;
//
//    [[nodiscard]] glm::vec3 getColor() const;
//    [[nodiscard]] static glm::vec3 getNextColor(const glm::vec3& base, const glm::vec3& delta, uint32_t& index);


//    [[nodiscard]] ConvexHull transformed(const glm::mat4& transform) const;
//    [[nodiscard]] std::vector<glm::vec2> sweptSection(const glm::vec3& origin, const glm::vec3& axis) const;

//    [[nodiscard]] bool raycast(const glm::vec3& origin, const glm::vec3& direction, float& t) const;
//    [[nodiscard]] bool raycast(const glm::vec3& origin, const glm::vec3& direction) const;

private:

    struct Triangle {
        uint32_t I0;
        uint32_t I1;
        uint32_t I2;

        uint32_t operator[](uint32_t i) const
        {
            switch(i){
                case 2:
                    return I2;
                case 1:
                    return I1;
                case 0:
                default:
                    return I0;
            }
        }

        uint32_t& operator[](uint32_t i)
        {
            switch(i){
                case 2:
                    return I2;
                case 1:
                    return I1;
                case 0:
                default:
                    return I0;
            }
        }
    };

    struct Facet {
        Triangle triangle;
        float *normal;
        std::vector<uint32_t> outside;
        std::vector<uint32_t> neighbors;
        bool onHull;
    };

    std::vector<Triangle> initialApproximation();

    void prepareFacets(const std::vector<Triangle>& triangles);
    void prepareFacets(const std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);
    void sortCloud(std::vector<uint32_t>& free, Facet& facet);
    void calculateHorizon(const float *apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);

    static bool isManifold(const std::vector<Facet> &facets);
    static bool isManifold(const Facet &dest, uint32_t src);

//    glm::vec3 normal(const Triangle& triangle);

private:

    VertexArray m_cloud;

    float* m_vertices;
    uint32_t m_vertexCount;

    uint32_t *m_facets;
    uint32_t *m_facetSizes;
    uint32_t m_facetCount;

    std::vector<Facet> facets;

    const static uint8_t STRIDE = 3;

};


//class ConvexDecomposition {
//public:
//
//    struct Settings {
//    public:
//        std::string name;
//        uint32_t maxHullCount;
//        uint32_t maxVertexCount;
//        float volumeError;
//        uint32_t voxelResolution;
//        uint32_t recurseDepth;
//
//        bool operator==(const Settings& settings) const {
//            return maxHullCount == settings.maxHullCount
//                   && maxVertexCount == settings.maxVertexCount
//                   && volumeError == settings.volumeError
//                   && voxelResolution == settings.voxelResolution
//                   && recurseDepth == settings.recurseDepth;
//        }
//
//        bool operator!=(const Settings& settings) const {
//            return !(*this == settings);
//        }
//    };
//
//    class ProcessLogger : public VHACD::IVHACD::IUserCallback, public VHACD::IVHACD::IUserLogger{
//    public:
//        ~ProcessLogger() override = default;
//
//        void Update(const double overallProgress, const double stageProgress, const char* const stage, const char *operation) final;
//        void NotifyVHACDComplete() final;
//
//        void Log(const char* const msg) final;
//
//    };
//
//    ConvexDecomposition(const Settings& settings = {"default", 24, 64, 1, 100000, 8});
//    ConvexDecomposition(const Settings& settings, const std::vector<glm::vec3>& vertices, const std::vector<Triangle>& triangles);
//
//    void applyNewSettings(const Settings& settings);
//    void restoreSettings();
//    void invalidate();
//    void decompose(const std::vector<glm::vec3>& vertices, const std::vector<Triangle>& triangles, bool async = true);
//    void decompose(float* vertices, uint32_t vertexCount, uint32_t* indices, uint32_t triangleCount, bool async = true);
//
//    [[nodiscard]] bool isValid() const;
//    [[nodiscard]] bool isLatest() const;
//    [[nodiscard]] const Settings& getActiveSettings() const;
//
//    [[nodiscard]] uint32_t getHullCount() const;
//    const std::vector<ConvexHull>& getHulls();
//
//private:
//    static VHACD::IVHACD::Parameters generateParameters(const Settings& settings);
//    void prepareConvexHulls(VHACD::IVHACD* iface);
//private:
//    bool m_Valid;
//    bool m_Latest;
//    Settings m_ActiveSettings;
//    Settings m_NewSettings;
//
//    std::vector<ConvexHull> m_Hulls;
//
//    glm::vec3 m_BaseColor;
//    glm::vec3 m_ColorDelta;
//};


#endif //PATHFINDER_CONVEXHULL_H