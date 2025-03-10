//
// Created by Cam on 2024-10-08.
//

#include "MeshHandler.h"

#include <assimp/Importer.hpp>      // C++ importer interface
//#include <assimp/Exporter.hpp>      // C++ exporter interface
#include <assimp/Exporter.hpp>      // C++ exporter interface

#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <iostream>

#include "core/Timer.h"

std::shared_ptr<Mesh> MeshHandler::loadAsMeshBody(const std::string& filepath, float scalar)
{
    ScopedTimer timer(filepath + " mesh loading");
    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile( filepath
                                                    ,  aiProcess_DropNormals
                                                    | aiProcess_JoinIdenticalVertices
                                                    | aiProcess_SortByPType);

    // Report failed imports
    if (nullptr == scene) {
        std::cout << "\033[31mImport failed: " << importer.GetErrorString() << "\033[0m\n";
        return nullptr;
    }

    uint32_t vertexCount = 0, faceCount = 0, indexCount = 0;
    for (uint32_t i = 0; i < scene->mNumMeshes; i++) {
        vertexCount += scene->mMeshes[i]->mNumVertices;

        faceCount += scene->mMeshes[i]->mNumFaces;
        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
            indexCount += scene->mMeshes[i]->mFaces[j].mNumIndices;
        }
    }

    float *vertices = new float[3 * vertexCount], *vPtr = vertices;
    uint32_t *faces = new uint32_t[3 * indexCount], *faceSizes = new uint32_t[faceCount], *fPtr = faces, *fsPtr = faceSizes, offset = 0;

    // Convert file content to a usable format
    for (uint32_t i = 0; i < scene->mNumMeshes; i++) {
        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
            *vPtr++ = scene->mMeshes[i]->mVertices[j].x * scalar;
            *vPtr++ = scene->mMeshes[i]->mVertices[j].y * scalar;
            *vPtr++ = scene->mMeshes[i]->mVertices[j].z * scalar;
        }

        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
            *fsPtr++ = scene->mMeshes[i]->mFaces[j].mNumIndices;
            for (uint32_t k = 0; k < scene->mMeshes[i]->mFaces[j].mNumIndices; k++) {
                *fPtr++ = scene->mMeshes[i]->mFaces[j].mIndices[k] + offset;
            }
        }

        offset += scene->mMeshes[i]->mNumVertices;
    }

    return std::make_shared<Mesh>(vertices, vertexCount, faces, faceSizes, faceCount);
}

void MeshHandler::exportMesh(const std::shared_ptr<Mesh>& mesh, const std::string& filepath)
{
    std::cout << "Exporting mesh...\n";
    ScopedTimer timer(filepath + " mesh export");
    Assimp::Exporter exporter;

    auto outputMesh = new aiMesh();
    outputMesh->mNumVertices = mesh->vertexCount();
    outputMesh->mVertices = new aiVector3f [3 * mesh->vertexCount()];
    memcpy(outputMesh->mVertices, mesh->vertices().data(), 3 * mesh->vertexCount() * sizeof(float));
    outputMesh->mNumFaces = mesh->faceCount();
    outputMesh->mFaces = new aiFace[mesh->faceCount()];
    outputMesh->mNormals = new aiVector3f[mesh->vertexCount()];
    memcpy(outputMesh->mNormals, mesh->vertexNormals().data(), 3 * mesh->vertexCount() * sizeof(float));
    auto idxPtr = mesh->faces().faces();
    for (uint32_t i = 0; i < mesh->faceCount(); i++) {
        outputMesh->mFaces[i].mNumIndices = mesh->faces().faceSizes()[i];
        outputMesh->mFaces[i].mIndices = new uint32_t[outputMesh->mFaces[i].mNumIndices];

        memcpy(outputMesh->mFaces[i].mIndices, idxPtr, outputMesh->mFaces[i].mNumIndices * sizeof(uint32_t));
        idxPtr += outputMesh->mFaces[i].mNumIndices;
    }
    outputMesh->mPrimitiveTypes = aiPrimitiveType_POLYGON;

    std::unique_ptr<aiScene> scene(new aiScene());
    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh * [] { outputMesh };
    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial * [] { new aiMaterial() };
    scene->mRootNode = new aiNode();
    scene->mRootNode->mNumMeshes = 1;
    scene->mRootNode->mMeshes = new unsigned [] { 0 };
    scene->mMetaData = new aiMetadata();


    exporter.Export(scene.get(), "objnomtl", filepath);
}