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

#include "modules/planning/scenarios/valet_parking/stage_parking.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::cyber::Clock;

StageResult StageParking::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  auto scenario_context = GetContextAs<ValetParkingContext>();
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;

  // With roi_type PULL_OVER, OpenSpaceRoiDecider reads planning_status.pull_over
  // for the target pose. Seed it with the parking spot's center + its HDMap
  // heading (NOT the ego heading, which would rotate the end node into the
  // adjacent obstacle -> "end_node in collision").
  hdmap::Id spot_id;
  spot_id.set_id(scenario_context->target_parking_spot_id);
  auto spot = hdmap::HDMapUtil::BaseMapPtr()->GetParkingSpaceById(spot_id);
  if (spot != nullptr) {
    const auto& pts = spot->polygon().points();
    common::math::Vec2d center(0.0, 0.0);
    double max_x = -std::numeric_limits<double>::max();
    for (const auto& p : pts) {
      center += p;
      max_x = std::max(max_x, p.x());
    }
    center /= static_cast<double>(pts.size());

    const auto& vehicle_param =
        common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
    // Park with the ego's front edge 0.3m before the spot's stop line (the
    // spot's max-x edge, which faces the road). The ego center is thus
    // front_edge_to_center behind that. This keeps the end node ON the road
    // (inside the PULL_OVER road ROI), unlike the spot center which sits off
    // the lane in the parking bay and fell out of XY bounds.
    const double stop_line_x = max_x;
    const double target_x =
        stop_line_x - 0.3 - vehicle_param.front_edge_to_center();

    auto* pull_over_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_pull_over();
    pull_over_status->mutable_position()->set_x(target_x);
    pull_over_status->mutable_position()->set_y(center.y());
    pull_over_status->set_theta(spot->parking_space().heading());
    // The pull-over ROI is built from these dimensions (GetPullOverSpot
    // constructs the target rectangle from them). Leaving them 0 makes the
    // rectangle degenerate and the ROI boundary excludes the spot, so the end
    // node falls out of XY bounds -> "end_node in collision". Seed them with
    // the ego's own box dimensions.
    pull_over_status->set_length_front(vehicle_param.front_edge_to_center());
    pull_over_status->set_length_back(vehicle_param.back_edge_to_center());
    pull_over_status->set_width_left(vehicle_param.left_edge_to_center());
    pull_over_status->set_width_right(vehicle_param.right_edge_to_center());
  }

  // Run OSP (Hybrid A* + IPOPT) into the spot. Don't return ERROR on task
  // failure — keep RUNNING so the planning framework isn't shut down.
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "[ValetParking][PARK] OSP task error, staying in PARK stage";
  }

  // Parked -> wait 5s (keep the ego stopped) -> PULL_OUT.
  if (IsParkingDone(*frame)) {
    if (!reached_spot_) {
      reached_spot_ = true;
      reached_time_ = Clock::NowInSeconds();
      AINFO << "[ValetParking][PARK] parked, waiting 5s";
    }
    if (Clock::NowInSeconds() - reached_time_ >= 5.0) {
      AINFO << "[ValetParking][PARK] waited 5s, pulling out";
      next_stage_ = "VALET_PARKING_PULL_OUT";
      return StageResult(StageStatusType::FINISHED);
    }
    return StageResult(StageStatusType::RUNNING);
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageParking::IsParkingDone(const Frame& frame) {
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  const double max_stop_speed = common::VehicleConfigHelper::Instance()
                                    ->GetConfig()
                                    .vehicle_param()
                                    .max_abs_speed_when_stopped();
  const bool is_stopped = std::fabs(adc_speed) < max_stop_speed;

  // 第一重：PostProcess 设置的 destination_reached 标志。
  if (frame.open_space_info().destination_reached() && is_stopped) {
    return true;
  }

  // 第二重：物理位置 + 停车状态兜底（车位中心距离 + 连续静止 tick）。
  auto* ctx = GetContextAs<ValetParkingContext>();
  if (ctx->target_parking_spot_id.empty()) {
    park_stop_tick_count_ = 0;
    return false;
  }
  if (!is_stopped) {
    park_stop_tick_count_ = 0;
    return false;
  }

  auto* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    park_stop_tick_count_ = 0;
    return false;
  }
  hdmap::Id spot_id;
  spot_id.set_id(ctx->target_parking_spot_id);
  auto spot = hdmap->GetParkingSpaceById(spot_id);
  if (spot == nullptr) {
    park_stop_tick_count_ = 0;
    return false;
  }
  const auto& pts = spot->polygon().points();
  if (pts.empty()) {
    park_stop_tick_count_ = 0;
    return false;
  }
  common::math::Vec2d center(0.0, 0.0);
  for (const auto& p : pts) {
    center += p;
  }
  center /= static_cast<double>(pts.size());

  const auto& vs = frame.vehicle_state();
  const common::math::Vec2d adc_xy(vs.x(), vs.y());
  static constexpr double kParkingDoneDistance = 2.0;  // meter
  const double dist_to_center = (adc_xy - center).Length();

  if (dist_to_center < kParkingDoneDistance) {
    static constexpr int kRequiredTicks = 5;
    if (++park_stop_tick_count_ >= kRequiredTicks) {
      AINFO << "[ValetParking][PARK] physical fallback: at spot center ("
            << dist_to_center << "m), stopped " << kRequiredTicks << " ticks";
      return true;
    }
  } else {
    park_stop_tick_count_ = 0;
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
