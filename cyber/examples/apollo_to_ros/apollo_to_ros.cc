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
#include "cyber/cyber.h"
#include "cyber/examples/apollo_to_ros/apollo_pointcloud.h"

using apollo::cyber::Time;

SendMessage::SendMessage() {}

SendMessage::~SendMessage() {
  listener_ = nullptr;
  listener_node_ = nullptr;
  Destroy();
}

void SendMessage::Init(uint16_t server_port) {
  AINFO << "SendMessage Init";
  bool bret = InitSocket(server_port);
  if (!bret) {
    AINFO << "Init Socket failed!";
    return;
  }
  running_ = true;
  async_accept_ = apollo::cyber::Async(&SendMessage::AcceptSocket, this);

  PointCloudReader();
}

void SendMessage::Destroy() {
  if (!running_) return;
  running_ = false;
  AINFO << "SendMessage Destroy";

  std::vector<std::shared_ptr<Session>> all_session;
  session_set_.GetAllData(all_session);
  for (auto& session : all_session) {
    session->Close();
  }
  session_set_.Clear();

  session_.Close();
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

  std::vector<std::shared_ptr<Session>> all_session;
  session_set_.GetAllData(all_session);
  for (auto& session : all_session) {
    bool bret = SendPointCloud(session, point_cloud);
    if (!bret) {
      session_set_.Erase(session);
    }
  }
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
  while (running_) {
    auto conn_session = session_.Accept((struct sockaddr*)nullptr, nullptr);
    session_set_.Insert(conn_session);
  }
}

int SendMessage::SessionSend(const std::shared_ptr<Session>& session,
                             const void* buf, size_t size) {
  size_t snd_size = 0;
  while (snd_size != size) {
    const char* data_buf = static_cast<const char*>(buf) + snd_size;
    int nbytes = static_cast<int>(session->Write(data_buf, size - snd_size));
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

bool SendMessage::SendPointCloud(
    const std::shared_ptr<Session>& session,
    const std::shared_ptr<PointCloud>& point_cloud) {
  uint32_t width = point_cloud->width();
  uint32_t height = point_cloud->height();
  ApolloPointCloud apollo_pointcloud(width, height);
  for (uint32_t i = 0; i < width * height; ++i) {
    apollo_pointcloud.points[i].x = point_cloud->point(i).x();
    apollo_pointcloud.points[i].y = point_cloud->point(i).y();
    apollo_pointcloud.points[i].z = point_cloud->point(i).z();
  }

  std::vector<char> snd_buf;
  apollo_pointcloud.Serialize(snd_buf);

  size_t size = snd_buf.size();
  // AINFO << "server send size:" << size;
  int nbytes = SessionSend(session, (const void*)(&size), sizeof(size_t));
  if (nbytes <= 0) {
    return false;
  }

  nbytes = SessionSend(session, snd_buf.data(), snd_buf.size());
  if (nbytes <= 0) {
    return false;
  }
  AINFO << "server data size:" << nbytes;
  return true;
}
