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

#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/path_decision.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::common::math::Vec2d;

bool StageApproachingParkingSpot::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir, void* context) {
  if (!Stage::Init(config, injector, config_dir, context)) {
    return false;
  }
  scenario_config_.CopyFrom(
      GetContextAs<ValetParkingContext>()->scenario_config);
  return true;
}

StageResult StageApproachingParkingSpot::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(frame);
  StageResult result;
  auto* ctx = GetContextAs<ValetParkingContext>();

  if (frame->reference_line_info().empty()) {
    AERROR << "[ValetParking][APPROACH] no reference line";
    return result.SetStageStatus(StageStatusType::ERROR);
  }
  if (ctx->target_parking_spot_id.empty()) {
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  // Project the cached routing anchor onto the reference line — that's where
  // the ADC stops before handing off to PARK's OSP. The fence stays on the
  // lane centerline, but the SL/EM path (LANE_FOLLOW_PATH) still nudges
  // laterally within the lane to dodge parked cars on the way there.
  auto& ref_line_info = frame->mutable_reference_line_info()->front();
  const auto& ref_line = ref_line_info.reference_line();
  SLPoint anchor_sl;
  const Vec2d anchor_xy(ctx->target_anchor_x, ctx->target_anchor_y);
  double target_stop_s = 0.0;
  if (ref_line.XYToSL(anchor_xy, &anchor_sl)) {
    target_stop_s = anchor_sl.s() - scenario_config_.approach_stop_setback();
  } else {
    // The routing anchor may sit off-lane (inside a parking bay) where the
    // reference line can't project it. Stop at the end of the current
    // reference line minus setback instead, so the ADC halts on the last
    // projectable part of the main lane.
    const double ref_line_length =
        ref_line.GetMapPath().accumulated_s().back();
    target_stop_s = ref_line_length - scenario_config_.approach_stop_setback();
    if (target_stop_s < 0.0) target_stop_s = 0.0;
    AWARN << "[ValetParking][APPROACH] cannot project anchor ("
          << anchor_xy.x() << ", " << anchor_xy.y()
          << ") onto reference line, falling back to ref_line end - setback = "
          << target_stop_s;
  }

  // IGNORE the parked cars in the SPEED (longitudinal) layer only. Their SL
  // boundary crosses l=0, so AddObstacle marks them "lane blocking" and builds
  // a reference-line ST boundary (the stop wall) that forces v=0 — even though
  // the path already nudges around them. A longitudinal-only ignore removes the
  // stop wall; we do NOT add a lateral ignore, so LANE_FOLLOW_PATH +
  // force_left_nudge still dodges them laterally.
  auto* path_decision = ref_line_info.path_decision();
  if (path_decision != nullptr) {
    static constexpr double kIgnoreRadius = 10.0;
    ObjectDecisionType ignore_decision;
    ignore_decision.mutable_ignore();
    for (const auto* obs : path_decision->obstacles().Items()) {
      if (obs == nullptr || obs->IsVirtual() || !obs->IsStatic()) {
        continue;
      }
      const auto& center = obs->PerceptionBoundingBox().center();
      if ((Vec2d(center.x(), center.y()) - anchor_xy).Length() > kIgnoreRadius) {
        continue;
      }
      path_decision->AddLongitudinalDecision(
          "ignore-speed-near-spot-in-valet-parking", obs->Id(), ignore_decision);
    }
  }

  const double adc_front_edge_s = ref_line_info.AdcSlBoundary().end_s();
  static constexpr double kFenceBuffer = 1.0;  // meters ahead of ADC minimum
  double stop_line_s = target_stop_s;
  // Only push the fence forward if the ADC has *already passed* the target.
  // Pushing whenever the fence is within kFenceBuffer of ADC lets the fence
  // slide along with a crawling ADC — infinite low-speed creep.
  if (target_stop_s < adc_front_edge_s) {
    stop_line_s = adc_front_edge_s + kFenceBuffer;
  }

  // Custom stop fence (a virtual brake obstacle) just before the spot.
  const std::string virtual_obstacle_id = "VALET_PARKING_BRAKE";
  const std::vector<std::string> wait_for_obstacle_ids;
  planning::util::BuildStopDecision(
      virtual_obstacle_id, stop_line_s, /*stop_distance=*/0.0,
      StopReasonCode::STOP_REASON_PULL_OVER, wait_for_obstacle_ids,
      "VALET_PARKING-scenario", frame, &ref_line_info);

  result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "[ValetParking][APPROACH] ref-line task error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  // Hand off to PARK once the ego has physically settled near target_stop_s
  // (world-frame displacement over time, not noisy instantaneous |v|).
  const auto& vs = injector_->vehicle_state()->vehicle_state();
  const Vec2d adc_xy_now(vs.x(), vs.y());
  static constexpr double kSettleRadius = 0.2;   // meters
  static constexpr int kSettleTicks = 10;        // @10Hz ≈ 1s of stillness
  const Vec2d settled_ref(settled_ref_x_, settled_ref_y_);
  const bool have_ref = settled_tick_count_ > 0;
  const double drift = have_ref ? (adc_xy_now - settled_ref).Length() : 0.0;
  if (!have_ref || drift > kSettleRadius) {
    settled_ref_x_ = adc_xy_now.x();
    settled_ref_y_ = adc_xy_now.y();
    settled_tick_count_ = 1;
  } else {
    settled_tick_count_++;
  }

  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  const bool at_target_and_settled =
      adc_front_edge_s + 0.5 >= target_stop_s &&
      settled_tick_count_ >= kSettleTicks;
  approach_tick_count_++;
  static constexpr int kApproachTimeoutTicks = 150;  // ~15s at 10Hz
  const bool timed_out = approach_tick_count_ >= kApproachTimeoutTicks;

  AINFO << "[ValetParking][APPROACH] target_stop_s=" << target_stop_s
        << " front_s=" << adc_front_edge_s << " v=" << adc_speed
        << " settled=" << settled_tick_count_ << "/" << kSettleTicks;

  // Hand off only once the ego has actually REACHED the stop fence and settled
  // there. The old "truly_stopped" shortcut (|v| < max_stop_speed) fired on a
  // v=0 ego that had merely failed to plan in LANE_FOLLOW and was still ~14m
  // short of the fence — that pushed PARK to start with a bad pose and a
  // too-far OpenSpace target, whose empty trajectory then crashed planning.
  if (at_target_and_settled || timed_out) {
    if (timed_out && !at_target_and_settled) {
      AINFO << "[ValetParking][APPROACH] timed out after "
            << approach_tick_count_ << " ticks, handing off to PARK";
    }
    next_stage_ = "VALET_PARKING_PARKING";
    return StageResult(StageStatusType::FINISHED);
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

}  // namespace planning
}  // namespace apollo
