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

#include "modules/planning/traffic_rules/traffic_light/traffic_light.h"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;

bool TrafficLight::Init(const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<TrafficLightConfig>(&config_);
}

Status TrafficLight::ApplyRule(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);

  return Status::OK();
}

void TrafficLight::MakeDecisions(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.enabled()) {
    return;
  }

  const auto& traffic_light_status =
      injector_->planning_context()->planning_status().traffic_light();

  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  // debug info
  planning_internal::SignalLightDebug* signal_light_debug =
      reference_line_info->mutable_debug()
          ->mutable_planning_data()
          ->mutable_signal_light();
  signal_light_debug->set_adc_front_s(adc_front_edge_s);
  signal_light_debug->set_adc_speed(
      injector_->vehicle_state()->linear_velocity());

  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info->reference_line().map_path().signal_overlaps();

  // 赛题八(交通灯路口减速通行):通过交通信号灯区域(路口)时速度不得超过 5 m/s。
  // 给每个路口(junction)overlap 加软限速,封顶 4.8 m/s(贴上限又不超速,留 0.2
  // 抗速度跟踪噪声),前后缓冲尽量短,让主车以最快速度通过路口又不超速;
  // 红灯时下方逻辑仍会建停止墙(速度 0 ≤ 5),两者不冲突。
  // 注意:必须放在下方"done"判断之前 —— 绿灯放行后该灯会被标记 done 并跳过,
  // 但此时主车正在通过路口,恰恰需要这个限速。
  // 地图没有 junction overlap 时,退回用 signal overlap 作为信号区域。
  static constexpr double kTrafficLightSpeedLimit = 4.8;  // 贴 5 m/s 上限,留 0.2 抗噪声
  static constexpr double kLimitBefore = 15.0;            // 路口前开始减速, m
  static constexpr double kLimitAfter = 3.0;              // 通过后尽快解除, m
  const auto& junction_overlaps =
      reference_line_info->reference_line().map_path().junction_overlaps();
  const auto& speed_limit_overlaps =
      junction_overlaps.empty() ? traffic_light_overlaps : junction_overlaps;
  for (const auto& overlap : speed_limit_overlaps) {
    if (overlap.end_s <= adc_back_edge_s) {
      continue;  // 已通过该区域,不再限速
    }
    reference_line_info->mutable_reference_line()->AddSpeedLimit(
        overlap.start_s - kLimitBefore, overlap.end_s + kLimitAfter,
        kTrafficLightSpeedLimit);
  }

  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    if (traffic_light_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    // check if traffic-light-stop already finished, set by scenario/stage
    bool traffic_light_done = false;
    for (const auto& done_traffic_light_overlap_id :
         traffic_light_status.done_traffic_light_overlap_id()) {
      if (traffic_light_overlap.object_id == done_traffic_light_overlap_id) {
        traffic_light_done = true;
        break;
      }
    }
    if (traffic_light_done) {
      continue;
    }

    // work around incorrect s-projection along round routing
    static constexpr double kSDiscrepanceTolerance = 10.0;
    const auto& reference_line = reference_line_info->reference_line();
    common::SLPoint traffic_light_sl;
    traffic_light_sl.set_s(traffic_light_overlap.start_s);
    traffic_light_sl.set_l(0);
    common::math::Vec2d traffic_light_point;
    reference_line.SLToXY(traffic_light_sl, &traffic_light_point);
    common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                        injector_->vehicle_state()->y()};
    const double distance =
        common::util::DistanceXY(traffic_light_point, adc_position);
    const double s_distance = traffic_light_overlap.start_s - adc_front_edge_s;
    ADEBUG << "traffic_light[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] s_distance["
           << s_distance << "] actual_distance[" << distance << "]";
    if (s_distance >= 0 &&
        fabs(s_distance - distance) > kSDiscrepanceTolerance) {
      ADEBUG << "SKIP traffic_light[" << traffic_light_overlap.object_id
             << "] close in position, but far away along reference line";
      continue;
    }

    auto signal_color =
        frame->GetSignal(traffic_light_overlap.object_id).color();
    const double stop_deceleration = util::GetADCStopDeceleration(
        injector_->vehicle_state(), adc_front_edge_s,
        traffic_light_overlap.start_s);
    ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] color["
           << signal_color << "] stop_deceleration[" << stop_deceleration
           << "]";

    // debug info
    planning_internal::SignalLightDebug::SignalDebug* signal_debug =
        signal_light_debug->add_signal();
    signal_debug->set_adc_stop_deceleration(stop_deceleration);
    signal_debug->set_color(signal_color);
    signal_debug->set_light_id(traffic_light_overlap.object_id);
    signal_debug->set_light_stop_s(traffic_light_overlap.start_s);

    // mayaochang add
    if (signal_color == perception::TrafficLight::GREEN ||
        signal_color == perception::TrafficLight::BLACK) {
      continue;
    }

    AINFO << "[traffic_light] id[" << traffic_light_overlap.object_id
          << "] color[" << signal_color << "] s_distance[" << s_distance << "]";

    // 决赛赛题一(红灯右转):红灯时主车在最右车道(右侧没有相邻车道),说明是要在
    // 这个路口右转,直接放行、不建停止墙,否则车会一直停在红灯前动不了。决赛地图
    // 没给右转车道标 turn(全是 NO_TURN),自带的无保护右转场景切不进来,所以改成
    // 直接查主车"右侧":红灯时右侧 3m 处查不到车道(在最右车道、右侧就是路沿),就
    // 认为主车是要右转。只对"红灯 + 右侧没车道"生效;直行主车右侧有相邻车道,仍
    // 照常停车。
    if (signal_color == perception::TrafficLight::RED) {
      const double heading = injector_->vehicle_state()->heading();
      const double x = injector_->vehicle_state()->x();
      const double y = injector_->vehicle_state()->y();

      // 右侧是否有车道:车头方向顺时针转 90° 就是右侧(ENU 下 heading 逆时针为正),
      // 在主车右侧 3m 处取查询点、半径 2m 内查车道。查不到 = 在最右车道 = 右转。
      static constexpr double kRightLateralOffset = 3.0;  // 右侧查询点横向距离 m
      static constexpr double kRightSearchRadius = 2.0;   // 查询半径 m
      common::PointENU right_point;
      right_point.set_x(x + std::cos(heading - M_PI_2) * kRightLateralOffset);
      right_point.set_y(y + std::sin(heading - M_PI_2) * kRightLateralOffset);
      std::vector<hdmap::LaneInfoConstPtr> right_lanes;
      const bool has_right_lane =
          HDMapUtil::BaseMap().GetLanes(right_point, kRightSearchRadius,
                                        &right_lanes) == 0 &&
          !right_lanes.empty();

      AINFO << "[traffic_light-red] id[" << traffic_light_overlap.object_id
            << "] heading[" << heading << "] x[" << x << "] y[" << y
            << "] right_lanes[" << right_lanes.size() << "]";

      if (!has_right_lane) {
        AINFO << "[traffic_light-red] no right lane, treat as right turn, "
                 "skip stop decision for traffic_light["
              << traffic_light_overlap.object_id << "]";
        continue;
      }
    }

    // Red/Yellow/Unknown: check deceleration
    if (stop_deceleration > config_.max_stop_deceleration()) {
      AWARN << "stop_deceleration too big to achieve.  SKIP red light";
      continue;
    }

    // build stop decision
    ADEBUG << "BuildStopDecision: traffic_light["
           << traffic_light_overlap.object_id << "] start_s["
           << traffic_light_overlap.start_s << "]"
           << "stop_distance: " << config_.stop_distance();
    std::string virtual_obstacle_id =
        TRAFFIC_LIGHT_VO_ID_PREFIX + traffic_light_overlap.object_id;
    const std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(
        virtual_obstacle_id, traffic_light_overlap.start_s,
        config_.stop_distance(), StopReasonCode::STOP_REASON_SIGNAL,
        wait_for_obstacles, Getname(), frame, reference_line_info);
  }
}

}  // namespace planning
}  // namespace apollo
