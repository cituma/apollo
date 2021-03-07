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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler.h"

using apollo::cyber::io::Session;

static inline int SessionRecv(Session& session, void* buf, int size) {
  int recv_size = 0;
  while (recv_size != size) {
    char* data_buf = static_cast<char*>(buf) + recv_size;
    int nbytes =
        static_cast<int>(session.Recv(data_buf, size - recv_size, 0));
    if (nbytes == 0) {
      AINFO << "server has been closed.";
      session.Close();
      return nbytes;
    }

    if (nbytes < 0) {
      AINFO << "receive message from server failed.";
      session.Close();
      return nbytes;
    }
    recv_size += nbytes;
  }

  return recv_size;
}

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  google::LogToStderr();

  size_t recv_buf_size = 10 * 1024 * 1024;

  uint16_t server_port = 11435;
  apollo::cyber::scheduler::Instance()->CreateTask(
      [&server_port, &recv_buf_size]() {
        struct sockaddr_in server_addr;
        server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons((uint16_t)server_port);

        std::string user_input;
        std::vector<char> recv_buf(recv_buf_size);

        Session session;
        session.Socket(AF_INET, SOCK_STREAM, 0);
        if (session.Connect((struct sockaddr*)&server_addr,
                            sizeof(server_addr)) < 0) {
          std::cout << "connect to server failed, " << strerror(errno)
                    << std::endl;
          return;
        }

        while (true) {
          size_t size = 0;
          int nbytes = SessionRecv(session, &size, sizeof(size_t));
          if (nbytes <= 0) return;

          if (size > recv_buf_size) {
            recv_buf_size = size;
            recv_buf.resize(size);
          }

          nbytes = SessionRecv(session, recv_buf.data(), size);
          if (nbytes <= 0) return;
          AINFO << "recv size:" << nbytes;
        }
      },
      "ros_client");

  apollo::cyber::WaitForShutdown();
  return 0;
}
