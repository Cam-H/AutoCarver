//
// Created by Cam on 2025-04-16.
//

#ifndef AUTOCARVER_PATHPLANNER_H
#define AUTOCARVER_PATHPLANNER_H

#include <vector>
#include <memory>
#include <glm.hpp>

class Robot;
class Scene;

#include "robot/trajectory/Trajectory.h"

class PathPlanner {
public:

//    void setScene(const std::shared_ptr<Scene>& scene);
//    void setRobot(const std::shared_ptr<Robot>& scene);

    virtual void addWaypoint(const glm::dmat4& transform);
    virtual void addWaypoint(const std::vector<double>& waypoint);

    bool prepareTrajectory(uint32_t w1Idx, uint32_t w2Idx);



protected:

    std::shared_ptr<Scene> scene;
    std::shared_ptr<Robot> robot;

    std::vector<std::vector<double>> m_waypoints;
    std::vector<Trajectory> m_trajectories;

};


#endif //AUTOCARVER_PATHPLANNER_H
