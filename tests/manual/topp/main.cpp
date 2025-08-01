
#include <vector>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "robot/trajectory/Waypoint.h"
#include "geometry/curves/PiecewisePolyPath.h"
#include "robot/trajectory/TOPP.h"

inline static std::string toString(double value, int precision)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

int main(int argc, char *argv[])
{

    std::vector<Waypoint> waypoints = {
            Waypoint({ 0, 20, 60 }),
            Waypoint({ 0, 90, 70 }),
            Waypoint({ 100, 10, 50 })
    };

    std::vector<double> vel_limits = {10.0, 5.0, 10.0};
    std::vector<double> acc_limits = {20.0, 10.0, 10.0};


    PiecewisePolyPath path(waypoints);
    TOPP solver(path, 20);

    solver.compute(vel_limits, acc_limits);
    solver.print();

    return 0;
}
