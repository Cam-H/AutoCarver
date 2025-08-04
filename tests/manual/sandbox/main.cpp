//
// Created by Cam on 2024-09-21.
//

#include "geometry/Mesh.h"
#include "fileIO/MeshHandler.h"

int main(int argc, char **argv)
{
    MeshHandler::loadAsMeshBody("../res/meshes/TurntableBase.obj");

    return 0;
}