//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_SCULPTPROCESS_H
#define AUTOCARVER_SCULPTPROCESS_H

#include "mcut/mcut.h"

#include "Scene.h"

#include "geometry/Mesh.h"

class SculptProcess : public Scene {
public:

    explicit SculptProcess(const std::shared_ptr<Mesh>& model);

private:

    std::shared_ptr<Mesh> model;
    std::shared_ptr<Mesh> sculpture;

    McContext context;

};


#endif //AUTOCARVER_SCULPTPROCESS_H
