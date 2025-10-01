#include "Collision3D.h"

template<class T1, class T2>
static bool test(const T1& bodyA, const T2& bodyB)
{
    try {
        auto simplex = GJK::solve3D(bodyA, bodyB, { 0, 0 });
        return simplex.colliding();
    } catch (std::exception& e) {
        std::cout << "\033[93m[Collision3D] Failed to test collision of two bodies\n" << e.what() << "\033[0m\n";
        return false;
    }
}

template<class T>
bool Collision3D::encloses(const T& body, const std::vector<glm::dvec3>& vertices)
{
    for (const glm::dvec3& vertex : vertices) {
        if (!encloses(body, vertex)) return false;
    }

    return true;
}

template<class T1, class T2>
EPA Collision3D::intersection(const T1& bodyA, const T2& bodyB, const std::pair<uint32_t, uint32_t>& idx)
{
    try {
        return EPA(bodyA, bodyB, GJK::solve3D(bodyA, bodyB, idx));
    } catch (std::exception& e) {
        std::cout << "\033[93m[Collision3D] Failed to calculate the intersection\n" << e.what() << "\033[0m\n";
        return {};
    }
}

template<class T1, class T2>
EPA Collision3D::intersection(const T1& bodyA, const T2& bodyB, const glm::dmat4& relative, const std::pair<uint32_t, uint32_t>& idx)
{
    try {
        return EPA(bodyA, bodyB, GJK::solve3D(bodyA, bodyB, relative, idx), relative);
    } catch (std::exception& e) {
        std::cout << "\033[93m[Collision3D] Failed to calculate the intersection\n" << e.what() << "\033[0m\n";
        return {};
    }
}