//
// Created by Cam on 2024-09-21.
//

#include <glm.hpp>

#include "geometry/Mesh.h"
#include "fileIO/MeshHandler.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/quaternion.hpp>

int main(int argc, char **argv)
{
    MeshHandler::loadAsMeshBody("../res/meshes/RobotBase.obj");

    glm::mat4 rotationMatrix(
            0.707f, -0.707f, 0.0f, 0.0f,
            0.707f,  0.707f, 0.0f, 0.0f,
            0.0f,    0.0f,   1.0f, 0.0f,
            0.0f,    0.0f,   0.0f, 1.0f
    );

    // Convert the rotation matrix to a quaternion
    glm::quat quat = glm::toQuat(rotationMatrix);
    quat = glm::quat_cast(rotationMatrix);
//    quat = glm::angleAxis(M_PI / 2, glm::dvec3{ 0, 1, 0 });

    // Extract the rotation angle and axis
    float angle = 2.0f * acos(quat.w);
    glm::vec3 axis = glm::normalize(glm::vec3(quat.x, quat.y, quat.z));

    std::cout << "Rotation Angle (radians): " << angle << std::endl;
    std::cout << "Rotation Axis (x, y, z): (" << axis.x << ", " << axis.y << ", " << axis.z << ")" << std::endl;


    return 0;
}