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

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  google::LogToStderr();

  uint16_t server_port = 11435;
  apollo::cyber::scheduler::Instance()->CreateTask(
      [&server_port]() {
        struct sockaddr_in server_addr;
        server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons((uint16_t)server_port);

        std::string user_input;
        std::vector<char> recv_buf(1024*1024);

        Session session;
        session.Socket(AF_INET, SOCK_STREAM, 0);
        if (session.Connect((struct sockaddr*)&server_addr,
                            sizeof(server_addr)) < 0) {
          std::cout << "connect to server failed, " << strerror(errno)
                    << std::endl;
          return;
        }

        while (true) {
          ssize_t nbytes = session.Recv(recv_buf.data(), recv_buf.size(), 0);

          if (nbytes == 0) {
            AINFO << "server has been closed.";
            session.Close();
            return;
          }

          if (nbytes < 0) {
            AINFO << "receive message from server failed.";
            session.Close();
            return;
          }

          AINFO << "recv size:" << nbytes;
        }
      },
      "ros_client");

  apollo::cyber::WaitForShutdown();
  return 0;
}
