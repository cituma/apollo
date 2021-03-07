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
#include "cyber/examples/apollo_to_ros/apollo_pointcloud.h"
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <thread>
#include "cyber/cyber.h"

using apollo::cyber::Time;

SendMessage::SendMessage() {}

SendMessage::~SendMessage() {
  listener_ = nullptr;
  listener_node_ = nullptr;
  Destroy();
}

void SendMessage::Init(uint16_t server_port) {
  AINFO << "SendMessage Init";
  running_ = true;
  InitSocket(server_port);
  async_accept_ = apollo::cyber::Async(&SendMessage::AcceptSocket, this);

  PointCloudReader();
}

void SendMessage::Destroy() {
  if (!running_) return;
  running_ = false;
  AINFO << "SendMessage Destroy";
  session_.Close();
  // for (auto& async_read : async_read_list_) {
  //  async_read.wait();
  //}
  async_accept_.wait();
}

void SendMessage::PointCloudReader() {
  listener_node_ = apollo::cyber::CreateNode("apollo_recv");
  // create listener
  listener_ = listener_node_->CreateReader<PointCloud>(
      "/apollo/sensor/velodyne64/compensator/PointCloud2",
      std::bind(&SendMessage::HandlePointCloud, this, std::placeholders::_1));
}

void SendMessage::HandlePointCloud(
    const std::shared_ptr<PointCloud>& point_cloud) {
  AINFO << "frame id:" << point_cloud->frame_id()
        << ", width:" << point_cloud->width()
        << ", height:" << point_cloud->height();

  queue_.Put(point_cloud);
}

bool SendMessage::InitSocket(uint16_t server_port) {
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htons(INADDR_ANY);
  server_addr.sin_port = htons(server_port);

  session_.Socket(AF_INET, SOCK_STREAM, 0);
  if (session_.Bind((struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    AINFO << "bind to port[" << server_port << "] failed.";
    return false;
  }

  if (session_.Listen(10) < 0) {
    AINFO << "socket listen failed.";
    return false;
  }
  return true;
}

void SendMessage::AcceptSocket() {
  // while(running_) {
  auto conn_session = session_.Accept((struct sockaddr*)nullptr, nullptr);
  AINFO << "Accepted";
  auto routine_name =
      "connected session" + std::to_string(Time::Now().ToNanosecond());
  // async_read_list_.push_back(apollo::cyber::Async(&SendMessage::SendSocket,
  // this, conn_session));
  SendSocket(conn_session);
  AINFO << "SendSocket end!";
  //}
}

int SendMessage::SessionSend(const std::shared_ptr<Session>& session,
                             const void* buf, size_t size) {
  size_t snd_size = 0;
  while(snd_size != size) {
    const char* data_buf = static_cast<const char*>(buf) + snd_size;
    int nbytes = static_cast<int>(session->Write(data_buf, size-snd_size));
    if (nbytes == 0) {
      AINFO << "client has been closed.";
      session->Close();
      return nbytes;
    }

    if (nbytes < 0) {
      AINFO << "send to client failed.";
      session->Close();
      return nbytes;
    }

    snd_size += nbytes;
  }

  return static_cast<int>(snd_size);
}

void SendMessage::SendSocket(const std::shared_ptr<Session>& session) {
  while (running_) {
    std::list<std::shared_ptr<PointCloud> > list;
    queue_.Take(list);

    for (auto point_cloud : list) {
#if 1
      uint32_t width = point_cloud->width();
      uint32_t height = point_cloud->height();
      ApolloPointCloud apollo_pointcloud(width, height);
      for(uint32_t i=0;i<width * height; ++i) {
        apollo_pointcloud.points[i].x = point_cloud->point(i).x();
        apollo_pointcloud.points[i].y = point_cloud->point(i).y();
        apollo_pointcloud.points[i].z = point_cloud->point(i).z();
      }

      std::vector<char> snd_buf;
      apollo_pointcloud.Serialize(snd_buf);

      size_t size = snd_buf.size();
      AINFO << "server send size:" << size;
      int nbytes = SessionSend(session, (const void*)(&size), sizeof(size_t));
      if (nbytes <= 0) return;

      nbytes =
          SessionSend(session, snd_buf.data(), snd_buf.size());
      if (nbytes <= 0) return;
      AINFO << "snd data size:" << nbytes;
#else
      std::string point_cloud_str;
      if (!point_cloud->SerializeToString(&point_cloud_str)) {
        AINFO << "SerializeToString failed!";
        continue;
      }
      size_t size = point_cloud_str.size();
      AINFO << "server send size:" << point_cloud_str.size();
      int nbytes = SessionSend(session, (const void*)(&size), sizeof(size_t));
      if (nbytes <= 0) return;

      nbytes =
          SessionSend(session, point_cloud_str.c_str(), point_cloud_str.size());
      if (nbytes <= 0) return;
#endif
    }
  }
}
