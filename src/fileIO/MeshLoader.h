//
// Created by Cam on 2024-10-08.
//

#ifndef AUTOCARVER_MESHLOADER_H
#define AUTOCARVER_MESHLOADER_H

#include "../geometry/Tesselation.h"

class MeshLoader {
public:

    static Tesselation loadAsTesselation(const std::string& filepath);


};


#endif //AUTOCARVER_MESHLOADER_H
