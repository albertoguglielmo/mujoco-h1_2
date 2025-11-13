#include "mjpc/tasks/h1_2/walk/h1_2_walk.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>
#include <fstream>

#include <stdexcept>

#include <cstdio>

#include "mjpc/task.h"
#include "mjpc/utilities.h"
//#include "mjpc/tasks/humanoid_bench/walk_reward.h"
#include "mujoco/mujoco.h"

#include "mjpc/tasks/h1_2_functions.h"

struct Point {
    double x, y, z;
};

namespace mjpc {

std::string H1_2_walk::XmlPath() const {
  return GetModelPath("h1_2/h1_2_walk.xml");
}
std::string H1_2_walk::Name() const { return "h1_2_walk"; }


// ------------------ Residuals for humanoid walk task ------------
//   Number of residuals:
//      Residual(0): torso height
//      Residual(1): pelvis-feet alignment
//      Residual(2): balance
//      Residual(3): upright
//      Residual(4): posture
//      Residual(5): face direction
//      Residual(6): walk
//      Residual(7): velocity
//      Residual(8): control
//      Residual(9): foot clearance
//      Residual(10): feet distance (3D)
//      Residual(11): knee
//   Number of parameters:
//      Parameter(0): torso height goal
//      Parameter(1): speed goal
//      Parameter(2): direction goal (in degrees)
//      Parameter(3): feet distance goal
//      Parameter(4): clearance goal
// ----------------------------------------------------------------

void H1_2_walk::ResidualFn::Residual(const mjModel *model, const mjData *data,
                                double *residual) const {
  // parameters
  double const torso_height_goal = parameters_[0];
  double const speed_goal = parameters_[1];
  double const direction_goal = parameters_[2];//fmod(360/10 * data->time, 360.0);
  double const feet_distance_goal = parameters_[3];
  double const clearance_goal =  parameters_[4];

  //double speed_goal= data->time * 0.2; // aumenta la velocità nel tempo
  int counter = 0;

  double* torso_position = SensorByName(model, data, "torso_position");
 

  // ----- torso height ----- //
  double torso_height = torso_position[2];
  residual[counter++] = torso_height - torso_height_goal;

  // ----- pelvis / feet ----- //
  double *foot_right = SensorByName(model, data, "foot_right");
  double *foot_left = SensorByName(model, data, "foot_left");
  double pelvis_height = SensorByName(model, data, "pelvis_position")[2];
  residual[counter++] =
      0.5 * (foot_left[2] + foot_right[2]) - pelvis_height - 0.2;

  // ----- balance ----- //
  // capture point
  double *subcom = SensorByName(model, data, "torso_subcom");
  double *subcomvel = SensorByName(model, data, "torso_subcomvel");

  double capture_point[3];
  mju_addScl(capture_point, subcom, subcomvel, 0.3, 3);
  capture_point[2] = 1.0e-3;

  // project onto line segment

  double axis[3];
  double center[3];
  double vec[3];
  double pcp[3];
  mju_sub3(axis, foot_right, foot_left);
  axis[2] = 1.0e-3;
  double length = 0.5 * mju_normalize3(axis) - 0.05;
  mju_add3(center, foot_right, foot_left);
  mju_scl3(center, center, 0.5);
  mju_sub3(vec, capture_point, center);

  // project onto axis
  double t = mju_dot3(vec, axis);

  // clamp
  t = mju_max(-length, mju_min(length, t));
  mju_scl3(vec, axis, t);
  mju_add3(pcp, vec, center);
  pcp[2] = 1.0e-3;

  // is standing
  double standing =   torso_height / mju_sqrt(torso_height * torso_height + 0.45 * 0.45) - 0.4;

  standing = std::max(0.0, standing);

  mju_sub(&residual[counter], capture_point, pcp, 2);
  mju_scl(&residual[counter], &residual[counter], standing, 2);

  counter += 2;

  // ----- upright ----- //
  double *torso_up = SensorByName(model, data, "torso_up");
  double *pelvis_up = SensorByName(model, data, "pelvis_up");
  double *foot_right_up = SensorByName(model, data, "foot_right_up");
  double *foot_left_up = SensorByName(model, data, "foot_left_up");
  double z_ref[3] = {0.0, 0.0, 1.0};

  // torso
  residual[counter++] = torso_up[2] - 1.0;

  // pelvis
  residual[counter++] = 0.3 * (pelvis_up[2] - 1.0);

  // right foot
  mju_sub3(&residual[counter], foot_right_up, z_ref);
  mju_scl3(&residual[counter], &residual[counter], 0.1 * standing);
  counter += 3;

  mju_sub3(&residual[counter], foot_left_up, z_ref);
  mju_scl3(&residual[counter], &residual[counter], 0.1 * standing);
  counter += 3;

  // ----- posture ----- //
  mju_sub(&residual[counter], data->qpos + 7, model->key_qpos + 7, model->nu);
  counter += model->nu;

  // ----- Walk ----- //
  double *torso_forward = SensorByName(model, data, "torso_forward");
  double *pelvis_forward = SensorByName(model, data, "pelvis_forward");
  double *foot_right_forward = SensorByName(model, data, "foot_right_forward");
  double *foot_left_forward = SensorByName(model, data, "foot_left_forward");

  double forward[2];
  mju_copy(forward, torso_forward, 2);
  mju_addTo(forward, pelvis_forward, 2);
  mju_addTo(forward, foot_right_forward, 2);
  mju_addTo(forward, foot_left_forward, 2);
  mju_normalize(forward, 2);

  // Face in right-direction
  double direction_goal_radiant = direction_goal * M_PI / 180;
  double face_x[2] = {cos(direction_goal_radiant), sin(direction_goal_radiant)};
  mju_sub(&residual[counter], forward, face_x, 2);
  mju_scl(&residual[counter], &residual[counter], standing, 2);
  counter += 2;

  // com vel
  double *waist_lower_subcomvel =
      SensorByName(model, data, "waist_lower_subcomvel");
  double *torso_velocity = SensorByName(model, data, "torso_velocity");
  double com_vel[2];
  mju_add(com_vel, waist_lower_subcomvel, torso_velocity, 2);
  mju_scl(com_vel, com_vel, 0.5, 2);

  // Walk forward
  residual[counter++] = standing * (mju_dot(com_vel, forward, 2) - speed_goal);

  // ----- move feet ----- //
  double *foot_right_vel = SensorByName(model, data, "foot_right_velocity");
  double *foot_left_vel = SensorByName(model, data, "foot_left_velocity");
  double move_feet[2];
  mju_copy(move_feet, com_vel, 2);
  mju_addToScl(move_feet, foot_right_vel, -0.5, 2);
  mju_addToScl(move_feet, foot_left_vel, -0.5, 2);

  mju_copy(&residual[counter], move_feet, 2);
  mju_scl(&residual[counter], &residual[counter], standing, 2);
  counter += 2;

  // ----- control ----- //
  mju_sub(&residual[counter], data->ctrl, model->key_qpos + 7,
          model->nu);  
  counter += model->nu;

  // ---- foot clearance ---- //
  residual[counter++] = clearance_goal -fabs(foot_left[2] - foot_right[2]);

  // ----- feet distance (3D) ----- //
  double feet_axis[3];
  mju_copy(feet_axis, foot_right, 3);
  mju_addToScl(feet_axis, foot_left, -1, 3);
  double feet_distance = mju_norm(feet_axis, 3);
  residual[counter++] = feet_distance - feet_distance_goal;

 // ----- knee ----- //
  double* left_joint_knwee = SensorByName(model, data, "left_knee_joint");
  double* right_joint_knwee = SensorByName(model, data, "right_knee_joint");
  residual[counter++] = fabs(left_joint_knwee[0]-0.15) + fabs(right_joint_knwee[0]-0.15);

   int user_sensor_dim = 0;
  for (int i = 0; i < model->nsensor; i++) {
    if (model->sensor_type[i] == mjSENS_USER) {
      user_sensor_dim += model->sensor_dim[i];
    }
  }
  if (user_sensor_dim != counter) {
    mju_error(
        "mismatch between total user-sensor dimension %d "
        "and actual length of residual %d",
        user_sensor_dim, counter);
  }
}
}