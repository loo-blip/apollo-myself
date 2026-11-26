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

#include "modules/planning/scenarios/valet_parking/stage_pull_out.h"

#include <algorithm>
#include <cmath>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::math::NormalizeAngle;

StageResult StagePullOut::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(frame);
  auto* ctx = GetContextAs<ValetParkingContext>();

  // OpenSpaceRoiDecider in PARK_AND_GO mode reads park_and_go.adc_init_position.
  // Seed it on our first tick or ROI_DECIDER returns "ADC initial position is
  // unavailable" every tick.
  if (!ctx->pull_out_init_done) {
    auto* pag_status = injector_->planning_context()
                           ->mutable_planning_status()
                           ->mutable_park_and_go();
    pag_status->Clear();
    auto* vs = injector_->vehicle_state();
    pag_status->mutable_adc_init_position()->set_x(vs->x());
    pag_status->mutable_adc_init_position()->set_y(vs->y());
    pag_status->mutable_adc_init_position()->set_z(0.0);
    pag_status->set_adc_init_heading(vs->heading());
    ctx->pull_out_init_done = true;
  }
  if (!ctx->pull_out_start_recorded) {
    auto* vs = injector_->vehicle_state();
    ctx->pull_out_start_x = vs->x();
    ctx->pull_out_start_y = vs->y();
    ctx->pull_out_start_recorded = true;
  }

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "[ValetParking][PULL_OUT] planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  if (IsReadyToHandOff(*frame)) {
    auto* scenario_context = GetContextAs<ValetParkingContext>();
    scenario_context->leave_finished_timestamp = frame->vehicle_state().timestamp();
    return FinishScenario();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StagePullOut::IsReadyToHandOff(const Frame& frame) const {
  if (frame.reference_line_info().empty()) {
    return false;
  }
  auto* vehicle_state_provider = injector_->vehicle_state();
  const common::VehicleState vehicle_state =
      vehicle_state_provider->vehicle_state();
  const common::math::Vec2d adc_xy(vehicle_state.x(), vehicle_state.y());

  // Hard gate: must have physically left the spot first, otherwise the first
  // tick passes all checks and PullOut terminates without executing any OSP.
  const auto* ctx = GetContextAs<ValetParkingContext>();
  static constexpr double kMinPullOutDistance = 15.0;  // meters
  if (ctx->pull_out_start_recorded) {
    const double moved = std::hypot(adc_xy.x() - ctx->pull_out_start_x,
                                    adc_xy.y() - ctx->pull_out_start_y);
    if (moved < kMinPullOutDistance) {
      return false;
    }
  } else {
    return false;
  }

  // 1. Back on a CITY_DRIVING lane.
  hdmap::LaneInfoConstPtr lane;
  double lane_s = 0.0;
  double lane_l = 0.0;
  auto adc_enu = common::util::PointFactory::ToPointENU(vehicle_state);
  if (hdmap::HDMapUtil::BaseMap().GetNearestLaneWithDistance(
          adc_enu, 2.0, &lane, &lane_s, &lane_l) != 0) {
    return false;
  }
  if (lane == nullptr || lane->lane().type() != hdmap::Lane::CITY_DRIVING ||
      !lane->IsOnLane({adc_xy.x(), adc_xy.y()})) {
    return false;
  }

  // 2. Heading aligned with the nearest reference line.
  const auto& ref_list = frame.reference_line_info();
  const auto ref_line_info = std::min_element(
      ref_list.begin(), ref_list.end(),
      [&](const ReferenceLineInfo& a, const ReferenceLineInfo& b) {
        common::SLPoint sl_a, sl_b;
        a.reference_line().XYToSL(adc_xy, &sl_a);
        b.reference_line().XYToSL(adc_xy, &sl_b);
        return std::fabs(sl_a.l()) < std::fabs(sl_b.l());
      });
  common::SLPoint adc_sl;
  ref_line_info->reference_line().XYToSL(adc_xy, &adc_sl);
  const auto ref_point =
      ref_line_info->reference_line().GetReferencePoint(adc_sl.s());
  const auto path_point = ref_point.ToPathPoint(adc_sl.s());
  const double heading_diff =
      std::fabs(NormalizeAngle(vehicle_state.heading() - path_point.theta()));
  static constexpr double kHeadingTolerance = 0.15;  // rad
  if (heading_diff > kHeadingTolerance) {
    return false;
  }

  // 3. Steering near-zero so the controller doesn't jerk on handover.
  static constexpr double kMaxSteerPct = 8.0;
  if (std::fabs(vehicle_state_provider->steering_percentage()) > kMaxSteerPct) {
    return false;
  }

  // 4. Lateral offset small.
  static constexpr double kLateralTolerance = 0.3;
  if (std::fabs(adc_sl.l()) > kLateralTolerance) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
