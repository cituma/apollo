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
#include <memory>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler_factory.h"
#include "cyber/task/task.h"

#include "modules/drivers/proto/pointcloud.pb.h"

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::drivers::PointCloud;
using apollo::cyber::io::Session;

template <typename T>
class SyncQueue {
 public:
  SyncQueue() : m_maxSize(32), m_needStop(false) {}
  SyncQueue(int maxSize) : m_maxSize(maxSize), m_needStop(false) {}

  void Put(const T& x) { Add(x); }
  void Put(const T&& x) { Add(std::forward<T>(x)); }

  void Take(std::list<T>& list) {
    std::unique_lock<std::mutex> locker(m_mutex);
    m_notEmpty.wait(locker, [this] { return (m_needStop || NotEmpty()); });
    if (m_needStop) return;
    list = std::move(m_queue);
    m_notFull.notify_one();
  }

  void Take(T& t) {
    std::unique_lock<std::mutex> locker(m_mutex);
    m_notEmpty.wait(locker, [this] { return (m_needStop || NotEmpty()); });
    if (m_needStop) return;
    t = m_queue.front();
    m_queue.pop_front();
    m_notFull.notify_one();
  }

  void Stop() {
    {
      std::lock_guard<std::mutex> locker(m_mutex);
      m_needStop = true;
    }
    m_notFull.notify_all();
    m_notEmpty.notify_all();
  }

  bool Empty() {
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_queue.empty();
  }

  bool Full() {
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_queue.size() == m_maxSize;
  }

  size_t Size() {
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_queue.size();
  }

  int Count() { return m_queue.size(); }

 private:
  bool NotFull() const { return (static_cast<int>(m_queue.size()) < m_maxSize); }
  bool NotEmpty() const { return !m_queue.empty(); }

  void Add(const T& x) {
    std::unique_lock<std::mutex> locker(m_mutex);
    m_notFull.wait(locker, [this] { return (m_needStop || NotFull()); });
    if (m_needStop) return;
    m_queue.push_back(x);
    m_notEmpty.notify_one();
  }

  template <typename F>
  void Add(F&& x) {
    std::unique_lock<std::mutex> locker(m_mutex);
    m_notFull.wait(locker, [this] { return (m_needStop || NotFull()); });
    if (m_needStop) return;
    m_queue.push_back(std::forward<F>(x));
    m_notEmpty.notify_one();
  }

 private:
  std::list<T> m_queue;
  std::mutex m_mutex;
  std::condition_variable m_notEmpty;
  std::condition_variable m_notFull;
  int m_maxSize;
  bool m_needStop;
};

class ApolloToRos : public Component<PointCloud> {
 public:
  ~ApolloToRos();

  bool Init() override;
  bool Proc(const std::shared_ptr<PointCloud>& point_cloud) override;

 private:
  void SendData(const std::shared_ptr<Session>& session);

 private:
  SyncQueue<std::shared_ptr<PointCloud> > queue_;
};
CYBER_REGISTER_COMPONENT(ApolloToRos)
