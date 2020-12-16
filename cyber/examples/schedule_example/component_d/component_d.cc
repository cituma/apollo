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
#include "cyber/examples/schedule_example/component_d/component_d.h"
#include <thread>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>

using apollo::cyber::Time;

bool ComponentD::Init() {
  AINFO << "ComponentD init";
  return true;
}

bool ComponentD::Proc(const std::shared_ptr<Driver>& msg0,
                      const std::shared_ptr<Driver>& msg1) {
  AINFO << "ComponentD read data:" << msg0->msg_id() << ", " << msg1->msg_id()
        << ". thread id:" << syscall(SYS_gettid); //std::this_thread::get_id();
  return true;
}
