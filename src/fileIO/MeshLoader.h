//
// Created by Cam on 2024-10-08.
//

#ifndef AUTOCARVER_MESHLOADER_H
#define AUTOCARVER_MESHLOADER_H

#include "../geometry/Tesselation.h"
#include "../geometry/Mesh.h"

class MeshLoader {
public:

    static Tesselation loadAsTesselation(const std::string& filepath);

    static std::shared_ptr<Mesh> loadAsMeshBody(const std::string& filepath, float scalar = 1.0f);

};


#endif //AUTOCARVER_MESHLOADER_H
