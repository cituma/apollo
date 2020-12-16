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
#include "cyber/examples/schedule_example/component_a/component_a.h"
#include <thread>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>

using apollo::cyber::Time;

bool ComponentA::Init() {
  AINFO << "ComponentA init";
  writer_ = node_->CreateWriter<Driver>("/apollo/output_a");
  return true;
}

bool ComponentA::Proc(const std::shared_ptr<Driver>& msg0) {
  uint64_t msg_id = msg0->msg_id();

  auto msg = std::make_shared<Driver>();
  msg->set_timestamp(Time::Now().ToNanosecond());
  msg->set_msg_id(msg_id + 1);
  msg->set_content(msg0->content());
  AINFO << "ComponentA read data:" << msg_id << ", write data:" << msg->msg_id()
        << ". thread id:" << syscall(SYS_gettid); //std::this_thread::get_id();
  writer_->Write(msg);
  return true;
}

