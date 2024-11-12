//
// Created by Cam on 2024-10-08.
//

#include "MeshLoader.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <iostream>

#include "core/Timer.h"

std::shared_ptr<Mesh> MeshLoader::loadAsMeshBody(const std::string& filepath, float scalar)
{
    ScopedTimer timer(filepath + " mesh loading");
    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile( filepath
                                                    , aiProcess_Triangulate
                                                    | aiProcess_DropNormals
                                                    | aiProcess_JoinIdenticalVertices
                                                    | aiProcess_SortByPType);

    // Report failed imports
    if (nullptr == scene) {
        std::cout << "\033[31mImport failed: " << importer.GetErrorString() << "\033[0m\n";
        return nullptr;
    }

    uint32_t vertexCount = 0, indexCount = 0;
    for (uint32_t i = 0; i < scene->mNumMeshes; i++) {
        vertexCount += scene->mMeshes[i]->mNumVertices;
        indexCount += scene->mMeshes[i]->mNumFaces;
    }

    float *vertices = new float[3 * vertexCount], *vPtr = vertices;
    uint32_t *indices = new uint32_t[3 * indexCount], *iPtr = indices, offset = 0;

    // Convert file content to a usable format
    for (uint32_t i = 0; i < scene->mNumMeshes; i++) {
        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
            *vPtr++ = scene->mMeshes[i]->mVertices[j].x * scalar;
            *vPtr++ = scene->mMeshes[i]->mVertices[j].y * scalar;
            *vPtr++ = scene->mMeshes[i]->mVertices[j].z * scalar;
        }

        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
            *iPtr++ = scene->mMeshes[i]->mFaces[j].mIndices[0] + offset;
            *iPtr++ = scene->mMeshes[i]->mFaces[j].mIndices[1] + offset;
            *iPtr++ = scene->mMeshes[i]->mFaces[j].mIndices[2] + offset;

            if (scene->mMeshes[i]->mFaces[j].mNumIndices != 3) std::cout << scene->mMeshes[i]->mFaces[j].mNumIndices << " - Face indices\n";
        }

        offset += scene->mMeshes[i]->mNumVertices;
    }

    return std::make_shared<Mesh>(vertices, vertexCount, indices, indexCount);
}