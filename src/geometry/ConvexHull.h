//
// Created by cameronh on 24/08/23.
//

#ifndef AUTOCARVER_CONVEXHULL_H
#define AUTOCARVER_CONVEXHULL_H

#include <QColor>

#include <string>
#include <memory>
#include <vector>

//#include "VHACD.h"

#include "VertexArray.h"
#include "FaceArray.h"
#include "Simplex.h"

class EPA;

class ConvexHull {
public:

    ConvexHull();
    ConvexHull(const float* cloud, uint32_t cloudSize);
    ConvexHull(VertexArray  cloud);

    ConvexHull(const ConvexHull& rhs) = default;
    ~ConvexHull();

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const VertexArray& vertices() const;

    [[nodiscard]] uint32_t facetCount() const;
    [[nodiscard]] const FaceArray& faces() const;

    [[nodiscard]] glm::vec3 facetNormal(uint32_t idx) const;

    [[nodiscard]] glm::vec3 center() const;

    [[nodiscard]] bool isSourceConvex() const;
    static bool isConvex(const VertexArray& test);

    [[nodiscard]] uint32_t walk(const glm::vec3& axis, uint32_t index = 0) const;

    [[nodiscard]] Simplex gjkIntersection(const ConvexHull& body, const glm::mat4& transform, std::pair<uint32_t, uint32_t>& idx) const;
    [[nodiscard]] EPA epaIntersection(const ConvexHull& body, const glm::mat4& transform, const glm::mat4& relativeTransform, std::pair<uint32_t, uint32_t>& idx) const;

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
        glm::vec3 normal;
        std::vector<uint32_t> outside;
        std::vector<uint32_t> neighbors;
        bool onHull;

//        ~Facet() { delete[] normal; }
    };

    std::vector<Triangle> initialApproximation();

    void prepareFacets(const std::vector<Triangle>& triangles);
    void prepareFacets(const std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);
    void sortCloud(std::vector<uint32_t>& free, Facet& facet);
    void calculateHorizon(const glm::vec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);

    glm::vec3 initialAxis(const ConvexHull& body, std::pair<uint32_t, uint32_t>& idx) const;
    glm::vec3 gjkSupport(const ConvexHull& body, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx) const;

    static bool isManifold(const std::vector<Facet> &facets);
    static bool isManifold(const Facet &dest, uint32_t src);

private:

    VertexArray m_cloud;

    VertexArray m_vertices;

    FaceArray m_faces;

//    uint32_t *m_facets;
//    uint32_t *m_facetSizes;
//    uint32_t m_facetCount;

    std::vector<glm::vec3> w_vertices;
    std::vector<Facet> facets;


    glm::vec3 m_center;
    std::vector<std::vector<uint32_t>> m_walks;

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