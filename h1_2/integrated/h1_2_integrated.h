// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MJPC_TASKS_H1_2_integreted
#define MJPC_TASKS_H1_2_integreted

#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

class H1_2_integrated_Task : public Task {
 public:
  class ResidualFn : public mjpc::BaseResidualFn {
   public:
    explicit ResidualFn(const H1_2_integrated_Task* task) : mjpc::BaseResidualFn(task) {}

// ------------------ Residuals for humanoid walk task ------------
//   Number of residuals:
//      Residual(0): 1 - humanoid_bench reward
//      Residual(1): torso height
//      Residual(2): pelvis-feet alignment
//      Residual(3): balance
//      Residual(4): upright
//      Residual(5): posture
//      Residual(6): face direction
//      Residual(7): walk
//      Residual(8): velocity
//      Residual(9): control
//      Residual(10): feet distance
//      Residual(11): foot clearance
//      Number of parameters:
//      Parameter(0): torso height goal
//      Parameter(1): speed_goal
//      Parameter(2): direction_goal
//      Parameter(3): feet_distance_goal
//      Parameter(4): desired_clearance
// ----------------------------------------------------------------

void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;
  };

  H1_2_integrated_Task() : residual_(this) {}

  std::string Name() const override;
  std::string XmlPath() const override;

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
};
}  // namespace mjpc

#endif  // MJPC_TASKS_HUMANOID_WALK_TASK_H_
