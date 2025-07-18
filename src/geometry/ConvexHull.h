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
#include "Triangle.h"
#include "Sphere.h"

class Mesh;
class EPA;

class ConvexHull {
public:

    ConvexHull();

    ConvexHull(const VertexArray&  cloud);
    ConvexHull(const std::vector<glm::vec3>&  cloud);

    ConvexHull(const std::shared_ptr<Mesh>& mesh);

    ConvexHull(const ConvexHull& rhs) = default;

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const std::vector<glm::vec3>& vertices() const;

    [[nodiscard]] uint32_t facetCount() const;
    [[nodiscard]] const FaceArray& faces() const;

    [[nodiscard]] glm::vec3 facetNormal(uint32_t idx) const;

    [[nodiscard]] glm::vec3 center() const;

    [[nodiscard]] bool empty() const;

    [[nodiscard]] uint32_t walk(const glm::vec3& axis, uint32_t index = 0) const;

    [[nodiscard]] std::vector<glm::vec3> border(const glm::vec3& faceNormal) const;

    [[nodiscard]] std::vector<uint32_t> horizon(const glm::vec3& axis) const;
    [[nodiscard]] std::vector<uint32_t> horizon(const glm::vec3& axis, const glm::vec3& support) const;

    template<class T>
    [[nodiscard]] Simplex gjkIntersection(const T& body, const glm::mat4& transform, std::pair<uint32_t, uint32_t>& idx) const
    {

        glm::vec3 axis = initialAxis(body, idx), next;
        Simplex simplex({ gjkSupport(body, transform, axis, idx), idx });

        axis = -simplex[0].val;

        int limit = (int)(vertexCount() + body.vertexCount());
        while (limit-- > 0) {
            next = gjkSupport(body, transform, axis, idx);

            // Check whether cs is impossible
            if (glm::dot (axis, next) <= 0) return simplex;

            simplex.add({ next, idx });

            // Check whether the cs is certain
            if (simplex.evaluate(axis)) return simplex;
        }

        std::cout << "CH GJK Error!\n";
        return Simplex(Simplex::Vertex{});
    }

    [[nodiscard]] EPA epaIntersection(const ConvexHull& body, const glm::mat4& transform, const glm::mat4& relativeTransform, std::pair<uint32_t, uint32_t>& idx) const;

    [[nodiscard]] std::vector<glm::vec3> intersection(const glm::vec3& origin, const glm::vec3& normal) const;

    [[nodiscard]] bool above(const glm::vec3& origin, const glm::vec3& normal) const;
    [[nodiscard]] bool below(const glm::vec3& origin, const glm::vec3& normal) const;
    [[nodiscard]] bool intersects(const glm::vec3& origin, const glm::vec3& normal) const;
    [[nodiscard]] bool intersects(const Sphere& sphere) const;

    [[nodiscard]] bool partition(const glm::vec3& origin, const glm::vec3& normal, std::vector<bool>& above) const;

    [[nodiscard]] ConvexHull fragment(const glm::vec3& origin, const glm::vec3& normal) const;
    [[nodiscard]] std::pair<ConvexHull, ConvexHull> fragments(const glm::vec3& origin, const glm::vec3& normal) const;

private:

    struct Facet {
        Triangle triangle;
        glm::vec3 normal;
        std::vector<uint32_t> outside;
        std::vector<uint32_t> neighbors;
        bool onHull;

//        ~Facet() { delete[] normal; }
    };

    void initialize();

    std::vector<Triangle> initialApproximation();
    glm::vec3 wNormal(const Triangle& triangle);

    [[nodiscard]] uint32_t step(const glm::vec3& normal, const glm::vec3& axis, uint32_t index) const;

    void prepareFacets(const std::vector<Triangle>& triangles);
    void prepareFacets(const std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);
    void sortCloud(std::vector<uint32_t>& free, Facet& facet);
    void calculateHorizon(const glm::vec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);

    glm::vec3 initialAxis(const Sphere& body, std::pair<uint32_t, uint32_t>& idx) const;
    glm::vec3 initialAxis(const ConvexHull& body, std::pair<uint32_t, uint32_t>& idx) const;

    glm::vec3 gjkSupport(const Sphere& body, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx) const;
    glm::vec3 gjkSupport(const ConvexHull& body, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx) const;

    std::vector<glm::vec3> intersection(const glm::vec3& origin, const glm::vec3& normal, const std::vector<bool>& above) const;


private:

    std::vector<glm::vec3> m_cloud;

    std::vector<glm::vec3> m_vertices;

    FaceArray m_faces;

//    std::vector<glm::vec3> w_vertices;
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