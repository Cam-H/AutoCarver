//
// Created by Cam on 2024-10-08.
//

#ifndef AUTOCARVER_MESHHANDLER_H
#define AUTOCARVER_MESHHANDLER_H

#include "../geometry/Mesh.h"

class MeshHandler {
public:

    static std::shared_ptr<Mesh> loadAsMeshBody(const std::string& filepath, double scalar = 1.0f);

    static void exportMesh(const std::shared_ptr<Mesh>& mesh, const std::string& filepath);

};


#endif //AUTOCARVER_MESHHANDLER_H
