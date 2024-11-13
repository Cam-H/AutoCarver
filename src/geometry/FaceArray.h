//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_FACEARRAY_H
#define AUTOCARVER_FACEARRAY_H

#include <cstdint>

class FaceArray {
public:
    FaceArray(const uint32_t* indices, uint32_t triangleCount);
    FaceArray(const uint32_t* faces, const uint32_t* faceSizes, uint32_t faceCount);

    FaceArray(const FaceArray &);
    FaceArray& operator=(const FaceArray &);

    FaceArray(FaceArray &&) noexcept;
    FaceArray& operator=(FaceArray &&) noexcept;

    ~FaceArray();

    uint32_t* operator[](uint32_t idx);
    uint32_t* operator[](uint32_t idx) const;

    [[nodiscard]] const uint32_t* faces() const;
    [[nodiscard]] const uint32_t* faceSizes() const;
    [[nodiscard]] uint32_t faceCount() const;

    void triangulation(uint32_t* indices); // Triangulates faces, presumes each face is convex
    [[nodiscard]] uint32_t triangleCount() const;


    [[nodiscard]] uint32_t size() const; // Get the size of the array in bytes

private:
    uint32_t *m_faces; // Loops of vertex indices that compose each face
    uint32_t *m_faceSizes; // Size of each individual face in vertices

    uint32_t m_faceCount; // Total number of faces
    uint32_t m_indexCount; // Total number of indices, the sum of all face sizes
};


#endif //AUTOCARVER_FACEARRAY_H
