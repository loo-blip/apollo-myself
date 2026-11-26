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

#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

#include <limits>
#include <vector>

#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::PathOverlap;
using apollo::routing::LaneWaypoint;

bool ValetParkingScenario::Init(std::shared_ptr<DependencyInjector> injector,
                                const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioValetParkingConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  init_ = true;
  return true;
}

bool ValetParkingScenario::IsTransferable(const Scenario* const other_scenario,
                                          const Frame& frame) {
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }

  // Do not re-trigger immediately after leaving; the spot the ego just vacated
  // would otherwise look free again and re-hijack the scenario.
  if (frame.vehicle_state().timestamp() - context_.leave_finished_timestamp <
      30.0) {
    return false;
  }

  PathOverlap target;
  if (!FindDockingParkingSpot(frame, &target)) {
    return false;
  }

  context_.target_parking_spot_id = target.object_id;
  // Cache the anchor so APPROACH can place a stop fence on the reference line
  // without re-reading planning_command.
  Vec2d anchor_cache;
  if (GetDockingAnchor(frame, &anchor_cache)) {
    context_.target_anchor_x = anchor_cache.x();
    context_.target_anchor_y = anchor_cache.y();
  }
  // The scenario context persists across transfers. Reset PULL_OUT stage state
  // so the next PULL_OUT re-seeds park_and_go.adc_init_position; otherwise a
  // previous cycle leaves pull_out_init_done=true and the ROI_DECIDER fails
  // with "ADC initial position is unavailable".
  context_.pull_out_init_done = false;
  context_.pull_out_start_recorded = false;
  context_.pull_out_start_x = 0.0;
  context_.pull_out_start_y = 0.0;
  AINFO << "ValetParking: taking over, parking_spot_id=" << target.object_id
        << " anchor=(" << context_.target_anchor_x << ", "
        << context_.target_anchor_y << ")";
  return true;
}

bool ValetParkingScenario::FindDockingParkingSpot(
    const Frame& frame, PathOverlap* out) const {
  Vec2d anchor;
  if (!GetDockingAnchor(frame, &anchor)) {
    ADEBUG << "ValetParking: routing has no usable docking anchor";
    return false;
  }

  // Range gate: only trigger when ADC is reasonably close to the anchor.
  const auto& vs = frame.vehicle_state();
  const Vec2d adc_xy(vs.x(), vs.y());
  const double range = context_.scenario_config.parking_spot_range_to_start();
  const double adc_to_anchor = (adc_xy - anchor).Length();
  if (adc_to_anchor > range) {
    return false;
  }

  // Go straight to HDMap to enumerate every spot near the anchor (the map path
  // may not expose the spot tied to the station).
  static constexpr double kSearchRadius = 12.0;
  auto anchor_enu =
      common::util::PointFactory::ToPointENU(anchor.x(), anchor.y(), 0.0);
  std::vector<hdmap::ParkingSpaceInfoConstPtr> spots;
  if (hdmap_->GetParkingSpaces(anchor_enu, kSearchRadius, &spots) != 0 ||
      spots.empty()) {
    AINFO << "ValetParking: no parking_space within " << kSearchRadius
          << "m of anchor";
    return false;
  }

  hdmap::ParkingSpaceInfoConstPtr best = nullptr;
  double best_d = std::numeric_limits<double>::infinity();
  for (const auto& spot : spots) {
    if (spot == nullptr) {
      continue;
    }
    const auto& pts = spot->polygon().points();
    if (pts.empty()) {
      continue;
    }
    Vec2d center(0.0, 0.0);
    for (const auto& p : pts) {
      center += p;
    }
    center /= static_cast<double>(pts.size());

    // Pick the spot closest to the DOCKING ANCHOR (the station waypoint), not
    // to the ADC — the ADC may still be ~25m away at trigger time, so
    // "closest to ADC" can pick the wrong bay.
    const double d = (center - anchor).Length();
    AINFO << "ValetParking: candidate " << spot->id().id()
          << " anchor_dist=" << d;
    if (d >= best_d) {
      continue;
    }
    if (IsParkingSpotOccupied(spot->id().id(), frame)) {
      AINFO << "ValetParking: skip occupied spot " << spot->id().id();
      continue;
    }
    best = spot;
    best_d = d;
  }

  if (best == nullptr) {
    AINFO << "ValetParking: every candidate spot around anchor is occupied";
    return false;
  }

  out->object_id = best->id().id();
  out->start_s = 0.0;
  out->end_s = 0.0;
  return true;
}

bool ValetParkingScenario::GetDockingAnchor(const Frame& frame,
                                            Vec2d* anchor) const {
  const auto& lv = frame.local_view();

  // Preferred: an intermediate waypoint from the original routing request
  // (the station). waypoint[0] is start, waypoint[n-1] is destination; anything
  // in between is a mandated pass-through point.
  if (lv.planning_command != nullptr &&
      lv.planning_command->has_lane_follow_command()) {
    const auto& lfc = lv.planning_command->lane_follow_command();
    if (lfc.has_routing_request()) {
      const auto& waypoints = lfc.routing_request().waypoint();
      if (waypoints.size() > 2) {
        const auto& vs = frame.vehicle_state();
        const Vec2d adc_xy(vs.x(), vs.y());
        const LaneWaypoint* best = nullptr;
        double best_d2 = std::numeric_limits<double>::infinity();
        for (int i = 1; i + 1 < waypoints.size(); ++i) {
          const auto& wp = waypoints.Get(i);
          if (!wp.has_pose()) {
            continue;
          }
          Vec2d p(wp.pose().x(), wp.pose().y());
          double d2 = (p - adc_xy).LengthSquare();
          if (d2 < best_d2) {
            best = &wp;
            best_d2 = d2;
          }
        }
        if (best != nullptr) {
          anchor->set_x(best->pose().x());
          anchor->set_y(best->pose().y());
          return true;
        }
      }
    }
  }

  // Fallback: routing end waypoint.
  if (lv.end_lane_way_point != nullptr) {
    anchor->set_x(lv.end_lane_way_point->pose().x());
    anchor->set_y(lv.end_lane_way_point->pose().y());
    return true;
  }
  return false;
}

bool ValetParkingScenario::IsParkingSpotOccupied(
    const std::string& parking_spot_id, const Frame& frame) const {
  hdmap::Id id;
  id.set_id(parking_spot_id);
  auto spot = hdmap_->GetParkingSpaceById(id);
  if (spot == nullptr) {
    return false;
  }

  // Do NOT shrink the spot polygon: any static obstacle that overlaps the spot
  // (even partially, e.g. a car head sticking into the first spot) makes it
  // unavailable. The correct spot is the second (free) one.
  const Polygon2d& spot_polygon = spot->polygon();

  for (const auto* obstacle : frame.obstacles()) {
    if (obstacle == nullptr || obstacle->IsVirtual() || !obstacle->IsStatic()) {
      continue;
    }
    if (spot_polygon.HasOverlap(obstacle->PerceptionPolygon())) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
