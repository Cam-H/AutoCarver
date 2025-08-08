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
#include "../geometry/Mesh.h"

std::shared_ptr<Mesh> MeshHandler::loadAsMeshBody(const std::string& filepath, double scalar)
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

//    TODO Development for separate node-based loading
//    std::vector<aiNode*> next = { scene->mRootNode }, nodes = {};
//    while (!next.empty()) {
//        nodes.emplace_back(next.back());
//        next.pop_back();
//
//        for (uint32_t i = 0; i < nodes.back()->mNumChildren; i++) {
//            next.emplace_back(nodes.back()->mChildren[i]);
//        }
//    }
//
//    for (const aiNode* node : nodes) {
//        std::cout << node << " " << node->mNumChildren << " " << node->mName.C_Str() << " " << node->mNumMeshes << " " << node->mParent << "\n";
//    }

    // Capture number of features
    uint32_t vertexCount = 0, faceCount = 0, indexCount = 0;
    for (uint32_t i = 0; i < scene->mNumMeshes; i++) {
        vertexCount += scene->mMeshes[i]->mNumVertices;
//        faceCount += scene->mMeshes[i]->mNumFaces;
//        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
//            indexCount += scene->mMeshes[i]->mFaces[j].mNumIndices;
//        }

//        std::cout << "TRAIT: " << scene->mMeshes[i]->mNumVertices << " " << scene->mMeshes[i]->mNumFaces << " " << indexCount << "\n";
    }

    // Capture materials
    std::vector<glm::dvec3> diffuseColors;
    for (uint32_t i = 0; i < scene->mNumMaterials; i++) {
        aiColor4D diffuse;
        aiGetMaterialColor(scene->mMaterials[i], AI_MATKEY_COLOR_DIFFUSE, &diffuse);
        diffuseColors.emplace_back(diffuse.r, diffuse.g, diffuse.b);
    }

    auto mesh = std::make_shared<Mesh>(vertexCount, 0, 0);

    uint32_t vIdx = 0, vOffset = 0, idxOffset = 0;

    // Convert file content to a usable format
    for (uint32_t i = 0; i < scene->mNumMeshes; i++) {

        // Capture vertex positions
        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
            mesh->m_vertices[vIdx++] = scalar * glm::dvec3(
                    scene->mMeshes[i]->mVertices[j].x,
                    scene->mMeshes[i]->mVertices[j].y,
                    scene->mMeshes[i]->mVertices[j].z
            );
        }

        // Capture faces
        for (uint32_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
            auto face = std::vector<uint32_t>(
                    scene->mMeshes[i]->mFaces[j].mIndices,
                    scene->mMeshes[i]->mFaces[j].mIndices + scene->mMeshes[i]->mFaces[j].mNumIndices);

            for (uint32_t& idx : face) idx += vOffset;

            mesh->m_faces.addFace(face, diffuseColors[scene->mMeshes[i]->mMaterialIndex]);
        }

        vOffset += scene->mMeshes[i]->mNumVertices;
        idxOffset += scene->mMeshes[i]->mNumFaces;
    }

    mesh->initialize();

    return mesh;
}

void MeshHandler::exportMesh(const std::shared_ptr<Mesh>& mesh, const std::string& filepath)
{

    if (mesh == nullptr) {
        std::cout << "Can not export! Null mesh provided\n";
        return;
    }

    std::cout << "Exporting mesh...\n";
    ScopedTimer timer(filepath + " mesh export");
    Assimp::Exporter exporter;

    auto outputMesh = new aiMesh();
    outputMesh->mNumVertices = mesh->vertexCount();
    outputMesh->mVertices = new aiVector3f [3 * mesh->vertexCount()];
    memcpy(outputMesh->mVertices, mesh->vertices().vertices().data(), 3 * mesh->vertexCount() * sizeof(double));
    outputMesh->mNumFaces = mesh->faceCount();
    outputMesh->mFaces = new aiFace[mesh->faceCount()];
    outputMesh->mNormals = new aiVector3f[mesh->vertexCount()];
    memcpy(outputMesh->mNormals, mesh->vertexNormals().vertices().data(), 3 * mesh->vertexCount() * sizeof(double));
    std::vector<uint32_t> indices = mesh->faces().faces();
    uint32_t idx = 0;
    for (uint32_t i = 0; i < mesh->faceCount(); i++) {
        outputMesh->mFaces[i].mNumIndices = mesh->faces().faceSizes()[i];
        outputMesh->mFaces[i].mIndices = new uint32_t[outputMesh->mFaces[i].mNumIndices];

        memcpy(outputMesh->mFaces[i].mIndices, &indices[idx], outputMesh->mFaces[i].mNumIndices * sizeof(uint32_t));
        idx += outputMesh->mFaces[i].mNumIndices;
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