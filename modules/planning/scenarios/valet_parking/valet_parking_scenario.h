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

#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/planning/scenarios/valet_parking/proto/valet_parking.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

struct ValetParkingContext : public ScenarioContext {
  ScenarioValetParkingConfig scenario_config;
  std::string target_parking_spot_id;
  bool pre_stop_rightaway_flag = false;
  hdmap::MapPathPoint pre_stop_rightaway_point;
  // Timestamp (simulation time) when the scenario finished leaving. Guards
  // IsTransferable against re-triggering on the spot the ego just vacated.
  double leave_finished_timestamp = 0.0;
  // World-frame anchor (routing docking waypoint) captured by IsTransferable.
  // APPROACH uses it to place its stop fence on the reference line.
  double target_anchor_x = 0.0;
  double target_anchor_y = 0.0;
  // PULL_OUT stage state (park-and-go handover tracking).
  bool pull_out_init_done = false;
  double pull_out_start_x = 0.0;
  double pull_out_start_y = 0.0;
  bool pull_out_start_recorded = false;
};

class ValetParkingScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;

  /**
   * @brief Get the scenario context.
   */
  ValetParkingContext* GetContext() override { return &context_; }

  bool IsTransferable(const Scenario* const other_scenario,
                      const Frame& frame) override;

 private:
  // Find the docking anchor (routing intermediate waypoint / end waypoint).
  bool GetDockingAnchor(const Frame& frame, common::math::Vec2d* anchor) const;
  // Find the nearest unoccupied parking spot around the anchor.
  bool FindDockingParkingSpot(const Frame& frame, hdmap::PathOverlap* out) const;
  // A spot is "occupied" if any static obstacle overlaps its (shrunk) polygon.
  bool IsParkingSpotOccupied(const std::string& parking_spot_id,
                             const Frame& frame) const;

  bool init_ = false;
  ValetParkingContext context_;
  const hdmap::HDMap* hdmap_ = nullptr;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ValetParkingScenario,
                                     Scenario)

}  // namespace planning
}  // namespace apollo
