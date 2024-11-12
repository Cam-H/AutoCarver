//
// Created by Cam on 2024-11-12.
//

#include "SculptProcess.h"

#include "Sculpture.h"

SculptProcess::SculptProcess(const std::shared_ptr<Mesh>& model)
    : Scene()
    , model(model)
    , sculpture(nullptr)
    , context(MC_NULL_HANDLE)
{
    auto sculpt = new Sculpture(model, 8, 8);
    prepareBody(sculpt);

    prepareBody(new Body(model));
//    prepareBody(sculpt->sculpture());

//    m_bodies[0].second->generate();
    std::cout << m_bodies.size() << "----\n";
    std::cout << m_bodies[0].second << "----\n";

}