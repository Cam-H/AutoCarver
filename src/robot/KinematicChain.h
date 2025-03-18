//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_KINEMATICCHAIN_H
#define AUTOCARVER_KINEMATICCHAIN_H

#include <vector>

#include "Joint.h"

class KinematicChain {
public:

    KinematicChain();

    void addJoint(Joint joint);

//    virtual

protected:
    std::vector<Joint> m_joints;
private:

};


#endif //AUTOCARVER_KINEMATICCHAIN_H
