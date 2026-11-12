/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file stage_intersection_cruise.cc
 **/

#include "modules/planning/scenarios/stop_sign_unprotected/stage_intersection_cruise.h"

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/scenarios/stop_sign_unprotected/context.h"

namespace apollo {
namespace planning {

StageResult StopSignUnprotectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  auto& reference_line_info = frame->mutable_reference_line_info()->front();

  hdmap::PathOverlap* traffic_sign_overlap =
      GetTrafficSignOverlap(reference_line_info, injector_->planning_context());
  if (!traffic_sign_overlap) {
    return FinishStage();
  }

  // The CREEP stage is skipped, so replace its creep wall with a soft speed
  // cap through the intersection. A wall forces a decel-to-zero profile (the
  // "又停一下"); AddSpeedLimit only caps the top speed, so the ego accelerates
  // to a low speed, takes the sharp turn without drifting out of lane, then
  // accelerates back to cruise.
  static constexpr double kSlowPassSpeed = 3.0;     // unit: m/s
  static constexpr double kLimitStartBuffer = 2.0;  // before stop line, m
  static constexpr double kLimitEndBuffer = 10.0;   // past overlap end, m
  reference_line_info.mutable_reference_line()->AddSpeedLimit(
      traffic_sign_overlap->start_s - kLimitStartBuffer,
      traffic_sign_overlap->end_s + kLimitEndBuffer, kSlowPassSpeed);

  // Keep yielding right-of-way while passing the intersection.
  reference_line_info.SetJunctionRightOfWay(traffic_sign_overlap->start_s,
                                            false);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  // Done once the ego's rear edge clears the stop sign by a safe margin. We
  // cannot reuse BaseStageCruise::CheckDone(): on maps with junction overlaps it
  // returns "done" while the ego is still before the stop line (it assumes the
  // ego already crept into the junction), which causes a
  // STOP <-> INTERSECTION_CRUISE oscillation.
  static constexpr double kIntersectionPassDist = 20.0;  // unit: m
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  if (adc_back_edge_s - traffic_sign_overlap->end_s >= kIntersectionPassDist) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StopSignUnprotectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

hdmap::PathOverlap*
StopSignUnprotectedStageIntersectionCruise::GetTrafficSignOverlap(
    const ReferenceLineInfo& reference_line_info,
    const PlanningContext* context) const {
  // stop_sign scenarios
  const auto& stop_sign_status = context->planning_status().stop_sign();
  const std::string traffic_sign_overlap_id =
      stop_sign_status.current_stop_sign_overlap_id();
  hdmap::PathOverlap* traffic_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          traffic_sign_overlap_id, ReferenceLineInfo::STOP_SIGN);
  return traffic_sign_overlap;
}

}  // namespace planning
}  // namespace apollo
