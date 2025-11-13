// tasks/h1_2_walk2/functions.h
#include "mjpc/tasks/h1_2_walk2/h1_2_walk.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>

#include <stdexcept>

#include <cstdio>

#include "mjpc/task.h"
#include "mjpc/utilities.h"
#include "mujoco/mujoco.h"

#include "mujoco/mujoco.h"
#include <utility>
#include <string>




struct Point3D {
    double x, y, z;
};


double tolerance(double x, std::pair<double,double> bounds = {0.0, 0.0},
                 double margin = 0.0,
                 std::string sigmoid_type = "gaussian",
                 double value_at_margin = 0.1);

double sigmoid(double x, double value_at_1, std::string sigmoid_type);

void SetGoalPosition(const mjModel* model, mjData* data, int mocap_id, double x, double y, double z);


std::vector<std::vector<Point3D>> MapPointCloudMatrix(const mjModel* model, const mjData* data);