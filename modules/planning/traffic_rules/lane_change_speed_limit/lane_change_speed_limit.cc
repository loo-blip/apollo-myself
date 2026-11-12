/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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
 * @file lane_change_speed_limit.cc
 **/

#include "modules/planning/traffic_rules/lane_change_speed_limit/lane_change_speed_limit.h"

#include <algorithm>

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

bool LaneChangeSpeedLimit::Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) {
    if (!TrafficRule::Init(name, injector)) {
        return false;
    }
    if (!TrafficRule::LoadConfig<LaneChangeSpeedLimitConfig>(&config_)) {
        return false;
    }
    lane_change_start_s_ = -1.0;
    lane_change_end_s_ = -1.0;
    prev_path_id_.clear();
    return true;
}

Status LaneChangeSpeedLimit::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
    const auto& change_lane_status = injector_->planning_context()->planning_status().change_lane();
    const auto status = change_lane_status.status();

    // 获取当前 ADC 在参考线上的位置
    const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
    common::SLPoint adc_sl;
    const auto& ref_line = reference_line_info->reference_line();
    double adc_s = 0.0;
    if (ref_line.XYToSL({vehicle_state.x(), vehicle_state.y()}, &adc_sl)) {
        adc_s = adc_sl.s();
    }

    const double limit = config_.speed_limit();

    // 兜底：只要存在变道参考线（横跨两条车道），就给所有参考线加限速。
    // 防止 status 被重置时短暂窗口内限速丢失导致超速。
    // 变道参考线 = routing 有 LEFT/RIGHT neighbor passage，正常转弯只有 FORWARD，不会触发。
    if (frame->reference_line_info().size() > 1) {
        reference_line_info->LimitCruiseSpeed(limit);
        auto* mutable_ref_line = reference_line_info->mutable_reference_line();
        mutable_ref_line->AddSpeedLimit(0.0, mutable_ref_line->Length(), limit);
        AINFO << "[LC_SPEED_LIMIT] active: limit=" << limit
              << " status=" << (change_lane_status.has_status() ? change_lane_status.status() : -1)
              << " ref_lines=" << frame->reference_line_info().size();
    }

    // 预减速：变道参考线已存在且等待变道时，提前限速防止进入 IN_CHANGE_LANE 后超速
    if (change_lane_status.has_status() &&
        change_lane_status.status() == ChangeLaneStatus::CHANGE_LANE_FINISHED &&
        frame->reference_line_info().size() > 1) {
        reference_line_info->LimitCruiseSpeed(limit);
        auto* mutable_ref_line = reference_line_info->mutable_reference_line();
        mutable_ref_line->AddSpeedLimit(0.0, mutable_ref_line->Length(), limit);
        return Status::OK();
    }

    if (change_lane_status.has_status()
        && (status == ChangeLaneStatus::IN_CHANGE_LANE || status == ChangeLaneStatus::CHANGE_LANE_FAILED
            || (status == ChangeLaneStatus::CHANGE_LANE_FINISHED && !change_lane_status.path_id().empty()))) {

        const auto& current_path_id = change_lane_status.path_id();
        const bool is_new_lane_change = (prev_path_id_ != current_path_id);

        if (status == ChangeLaneStatus::IN_CHANGE_LANE) {
            // 新变道：重置起止位置
            if (is_new_lane_change) {
                lane_change_start_s_ = adc_s;
                lane_change_end_s_ = -1.0;
                prev_path_id_ = current_path_id;
            }
            // 变道中：从变道起点前 5m 到参考线末端限速
            const double limit_start_s = std::max(0.0, lane_change_start_s_ - kRegionBuffer);
            reference_line_info->LimitCruiseSpeed(limit);
            auto* mutable_ref_line = reference_line_info->mutable_reference_line();
            mutable_ref_line->AddSpeedLimit(limit_start_s, mutable_ref_line->Length(), limit);
            return Status::OK();
        }

        if (status == ChangeLaneStatus::CHANGE_LANE_FINISHED) {
            if (is_new_lane_change) {
                lane_change_end_s_ = adc_s;
            }
            // 变道完成后 5m 内继续限速
            if (adc_s < lane_change_end_s_ + kRegionBuffer) {
                reference_line_info->LimitCruiseSpeed(limit);
                auto* mutable_ref_line = reference_line_info->mutable_reference_line();
                mutable_ref_line->AddSpeedLimit(0.0, lane_change_end_s_ + kRegionBuffer, limit);
                return Status::OK();
            }
        }

        // CHANGE_LANE_FAILED: 也限速（重试期间）
        reference_line_info->LimitCruiseSpeed(limit);
        auto* mutable_ref_line = reference_line_info->mutable_reference_line();
        mutable_ref_line->AddSpeedLimit(0.0, mutable_ref_line->Length(), limit);
        return Status::OK();
    }

    // 非变道状态，重置起止位置跟踪
    lane_change_start_s_ = -1.0;
    lane_change_end_s_ = -1.0;
    return Status::OK();
}

}  // namespace planning
}  // namespace apollo
