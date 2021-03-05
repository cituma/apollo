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
#include "cyber/examples/apollo_to_ros/apollo_to_ros.h"
#include <thread>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>

using apollo::cyber::Time;

bool ApolloToRos::Init() {
  AINFO << "ApolloToRos init";
  return true;
}

bool ApolloToRos::Proc(const std::shared_ptr<PointCloud>& point_cloud) {
  AINFO << "frame id:" << point_cloud->frame_id() << ", width:" << point_cloud->width()
        << ", height:" << point_cloud->height();
  return true;
}
