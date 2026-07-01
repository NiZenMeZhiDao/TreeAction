#pragma once

#include <memory>
#include <vector>

namespace r2_bt
{

struct FinalWaypoint
{
  double target_x = 0.0;
  double target_y = 0.0;
  double target_yaw = 0.0;
  int pid_profile = 1;
  double max_vel = 0.0;
  double max_wz = 0.0;
  double timeout_sec = 30.0;
};

using FinalWaypointList = std::vector<FinalWaypoint>;
using FinalWaypointListPtr = std::shared_ptr<FinalWaypointList>;

}  // namespace r2_bt
