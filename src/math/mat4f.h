//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_MAT4F_H
#define AUTOCARVER_MAT4F_H

#include "vec3f.h"

class mat4f {
public:
    mat4f(float scalar = 1.0f);

    mat4f(vec3f position);

public:
    float mat[4][4];
};


#endif //AUTOCARVER_MAT4F_H
