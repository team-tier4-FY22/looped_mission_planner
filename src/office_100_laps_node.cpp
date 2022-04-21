// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "office_100_laps/office_100_laps.hpp"

#include <cmath>
#include <memory>
#include <string>

OPffice100Laps::OPffice100Laps()
: Node("office_100_laps"),
  output_frame_(declare_parameter("base_link", "base_link")),
  message_timeout_sec_(declare_parameter("message_timeout_sec", 0.2))
{
}

OPffice100Laps::~OPffice100Laps() {}

void OPffice100Laps::callbackOdometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr)
{
    
}

void OPffice100Laps::timerCallback()
{
}