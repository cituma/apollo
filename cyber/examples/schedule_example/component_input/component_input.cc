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
#include "cyber/examples/schedule_example/component_input/component_input.h"
#include <thread>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>

using apollo::cyber::Time;

bool ComponentInput::Init() {
  AINFO << "ComponentInput init";
  writer_ = node_->CreateWriter<Driver>("/apollo/input");
  async_result_ = apollo::cyber::Async(&ComponentInput::run, this);
  return true;
}

ComponentInput::~ComponentInput() {
  async_result_.wait();
}

#if 0
bool ComponentInput::Proc(const std::shared_ptr<Driver>& msg0) {
  uint64_t msg_id = msg0->msg_id();

  auto msg = std::make_shared<Driver>();
  msg->set_timestamp(Time::Now().ToNanosecond());
  msg->set_msg_id(msg_id + 1);
  msg->set_content(msg0->content());
  AINFO << "ComponentInput read data:" << msg_id << ", write data:" << msg->msg_id()
        << ". thread id:" << std::this_thread::get_id();
  writer_->Write(msg);
  return true;
}
#endif

void ComponentInput::run() {
  AINFO << "ComponentInput::run start";
  auto msg = std::make_shared<Driver>();
  for (uint64_t i = 0; i < 20; ++i) {
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_msg_id(i);
    msg->set_content("schedule example");
    AINFO << "ComponentInput write data:" << msg->msg_id()
          << ". thread id:" << syscall(SYS_gettid);
    writer_->Write(msg);

    apollo::cyber::SleepFor(std::chrono::microseconds(500 * 1000)); //sleep 500ms
  }
  AINFO << "ComponentInput::run end";
}
