//
// Created by Cam on 2024-10-08.
//

#include "MeshLoader.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <iostream>


Tesselation MeshLoader::loadAsTesselation(const std::string& filepath)
{
    Tesselation tessel;

    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile( filepath,
                                              aiProcess_CalcTangentSpace       |
                                              aiProcess_Triangulate            |
                                              aiProcess_JoinIdenticalVertices  |
                                              aiProcess_SortByPType);

    // Report failed imports
    if (nullptr == scene) {
        std::cout << "Import failed: " << importer.GetErrorString() << "\n";
        return tessel;
    }

    // Convert file content to a usable format
    std::cout << "Assimp: " << scene->mNumMeshes << " " << scene->mNumMaterials << "\n";
    for (uint32_t i = 0; i < scene->mNumMeshes; i++) {
        std::vector<QVector3D> vertices;
        std::vector<Triangle> triangles;

        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
            vertices.emplace_back(scene->mMeshes[i]->mVertices[j].x, scene->mMeshes[i]->mVertices[j].y, scene->mMeshes[i]->mVertices[j].z);
        }

        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
            triangles.emplace_back((uint32_t)scene->mMeshes[i]->mFaces[j].mIndices[0], (uint32_t)scene->mMeshes[i]->mFaces[j].mIndices[1], (uint32_t)scene->mMeshes[i]->mFaces[j].mIndices[2]);
            if (scene->mMeshes[i]->mFaces[j].mNumIndices != 3) std::cout << scene->mMeshes[i]->mFaces[j].mNumIndices << " - Face indices\n";
        }

        tessel.append(vertices, triangles);
    }



    std::cout << filepath << " xa\n";

    return tessel;
}