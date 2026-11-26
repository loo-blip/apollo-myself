/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

namespace apollo {
namespace planning {

// APPROACH: native SL/EM on-lane cruise. Ignores static obstacles near the
// parking spot (they are PARK's OpenSpace problem, not the approach's), places
// a stop fence on the lane centerline just before the spot, and hands off to
// PARK once the ego has physically settled there.
class StageApproachingParkingSpot : public Stage {
 public:
  bool Init(const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir, void* context);
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

 private:
  ScenarioValetParkingConfig scenario_config_;
  // Physical-settling detection (world-frame displacement, not noisy |v|).
  double settled_ref_x_ = 0.0;
  double settled_ref_y_ = 0.0;
  int settled_tick_count_ = 0;
  int approach_tick_count_ = 0;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::StageApproachingParkingSpot, Stage)

}  // namespace planning
}  // namespace apollo
