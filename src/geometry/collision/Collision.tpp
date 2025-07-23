#include "Collision.h"

template<class T1, class T2>
bool Collision::test(const T1& bodyA, const T2& bodyB)
{
    try {
        auto simplex = gjk(bodyA, bodyB, { 0, 0 });
        return simplex.colliding();
    } catch (std::exception& e) {
        std::cout << "\033[93m[Collision] Failed to test collision of two bodies\n" << e.what() << "\033[0m\n";
        return false;
    }
}

template<class T1, class T2>
Simplex Collision::gjk(const T1& bodyA, const T2& bodyB, const std::pair<uint32_t, uint32_t>& idx)
{
    if (!bodyA.isValid() || !bodyB.isValid()) throw std::runtime_error("[Collision] Invalid bodies. Can not calculate GJK");

    glm::vec3 axis = { 1, 0, 0 };
    Simplex simplex(EPA::gjkSupport(bodyA, bodyB, axis, idx));

    axis = -simplex[0].val;

    int limit = 1e3;
    while (limit-- > 0) {
        const auto& [aIdx, bIdx, next] = EPA::gjkSupport(bodyA, bodyB, axis, simplex[0].idx); // Use most recent idx

        // Check whether cs is impossible
        if (glm::dot (axis, next) < 0) return simplex;

        simplex.add(Simplex::Vertex({ aIdx, bIdx }, next));

        // Check whether the cs is certain
        if (simplex.evaluate(axis)) return simplex;
    }

    throw std::runtime_error("[Collision] Failed to terminate when executing GJK");
}

template<class T1, class T2>
Simplex Collision::gjk(const T1& bodyA, const T2& bodyB, const glm::mat4& transform, const std::pair<uint32_t, uint32_t>& idx)
{
    if (!bodyA.isValid() || !bodyB.isValid()) throw std::runtime_error("[Collision] Invalid bodies. Can not calculate GJK");

    glm::vec3 axis = { 1, 0, 0 };
    Simplex simplex(EPA::gjkSupport(bodyA, bodyB, axis, transform, idx));

    axis = -simplex[0].val;

    int limit = 1e3;
    while (limit-- > 0) {
        const auto& [aIdx, bIdx, next] = EPA::gjkSupport(bodyA, bodyB, axis, transform, simplex[0].idx); // Use most recent idx

        // Check whether cs is impossible
        if (glm::dot(axis, next) < 0) return simplex;

        simplex.add(Simplex::Vertex({ aIdx, bIdx }, next));

        // Check whether the cs is certain
        if (simplex.evaluate(axis)) return simplex;
    }

    throw std::runtime_error("[Collision] Failed to terminate when executing GJK");
}

template<class T1, class T2>
glm::vec3 Collision::initialAxis(const T1& bodyA, const T2& bodyB)
{
    return bodyA.start() - bodyB.start();
}

template<class T>
bool Collision::encloses(const T& body, const std::vector<glm::vec3>& vertices)
{
    for (const glm::vec3& vertex : vertices) {
        if (!encloses(body, vertex)) return false;
    }

    return true;
}

template<class T1, class T2>
EPA Collision::intersection(const T1& bodyA, const T2& bodyB, const std::pair<uint32_t, uint32_t>& idx)
{
    try {
        return EPA(bodyA, bodyB, gjk(bodyA, bodyB, idx));
    } catch (std::exception& e) {
        std::cout << "\033[93m[Collision] Failed to calculate the intersection\n" << e.what() << "\033[0m\n";
        return {};
    }
}

template<class T1, class T2>
EPA Collision::intersection(const T1& bodyA, const T2& bodyB, const glm::mat4& relative, const std::pair<uint32_t, uint32_t>& idx)
{
    try {
        return EPA(bodyA, bodyB, gjk(bodyA, bodyB, relative, idx), relative);
    } catch (std::exception& e) {
        std::cout << "\033[93m[Collision] Failed to calculate the intersection\n" << e.what() << "\033[0m\n";
        return {};
    }
}