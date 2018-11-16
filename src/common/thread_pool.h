#pragma once

#include <deque>
#include <functional>
#include <thread>
#include <vector>

#include "mutex.h"

namespace static_map {
namespace common {

// A fixed number of threads working on a work queue of work items. Adding a
// new work item does not block, and will be executed by a background thread
// eventually. The queue must be empty before calling the destructor. The thread
// pool will then wait for the currently executing work items to finish and then
// destroy the threads.
// 线程池
// 只传入需要创建的线程个数，后台创建若干个 std::thread
// 主要的函数只有一个 —— Schedule，主要功能是将传入的功能函数添加到待工作的队列中
// 每个线程都会循环不断的获取队列中未被执行的函数进行执行
class ThreadPool {
 public:
  explicit ThreadPool(int num_threads);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  void Schedule(const std::function<void()>& work_item);

 private:
  void DoWork();

  Mutex mutex_;
  bool running_ GUARDED_BY(mutex_) = true;
  std::vector<std::thread> pool_ GUARDED_BY(mutex_);
  std::deque<std::function<void()>> work_queue_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace static_map

