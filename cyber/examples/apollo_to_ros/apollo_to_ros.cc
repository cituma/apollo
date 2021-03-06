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
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <thread>

using apollo::cyber::Time;

ApolloToRos::~ApolloToRos() {}

bool ApolloToRos::Init() {
  AINFO << "ApolloToRos init";

  uint16_t server_port = 11435;
  apollo::cyber::scheduler::Instance()->CreateTask(
      [&server_port]() {
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htons(INADDR_ANY);
        server_addr.sin_port = htons(server_port);

        Session session;
        session.Socket(AF_INET, SOCK_STREAM, 0);
        if (session.Bind((struct sockaddr*)&server_addr, sizeof(server_addr)) <
            0) {
          std::cout << "bind to port[" << server_port << "] failed."
                    << std::endl;
          return;
        }
        session.Listen(10);
        auto conn_session = session.Accept((struct sockaddr*)nullptr, nullptr);
        AINFO << "accepted";
        auto routine_name =
            "connected session" + std::to_string(Time::Now().ToNanosecond());
        apollo::cyber::scheduler::Instance()->CreateTask(
            std::bind(&ApolloToRos::SendData, conn_session), routine_name);
      },
      "to_ros_server");

  return true;
}

bool ApolloToRos::Proc(const std::shared_ptr<PointCloud>& point_cloud) {
  AINFO << "frame id:" << point_cloud->frame_id()
        << ", width:" << point_cloud->width()
        << ", height:" << point_cloud->height();
  queue_.Put(point_cloud);
  return true;
}

void ApolloToRos::SendData(const std::shared_ptr<Session>& session) {
  std::list<std::shared_ptr<PointCloud> > list;
  queue_.Take(list);

  for (auto point_cloud : list) {
    std::string point_cloud_str;
    if (!point_cloud->SerializeToString(&point_cloud_str)) {
      AINFO << "SerializeToString failed!";
      return;
    }
    int nbytes = static_cast<int>(
        session->Write(point_cloud_str.c_str(), point_cloud_str.size()));
    if (nbytes == 0) {
      AINFO << "client has been closed.";
      session->Close();
    }

    if (nbytes < 0) {
      AINFO << "send to client failed.";
      session->Close();
    }
  }
}
