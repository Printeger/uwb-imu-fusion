#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace uwb_imu_pl {

struct CandidateWorkerPool::Impl {
  explicit Impl(std::size_t count) : scratches(count) {
    if (count == 0) throw std::invalid_argument("candidate worker count is zero");
    threads.reserve(count);
    for (std::size_t worker = 0; worker < count; ++worker) {
      threads.emplace_back([this, worker] { loop(worker); });
    }
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopping = true;
      ++generation;
    }
    wake.notify_all();
    for (auto& thread : threads) if (thread.joinable()) thread.join();
  }

  void loop(std::size_t worker) {
    std::size_t observed_generation = 0;
    std::unique_lock<std::mutex> lock(mutex);
    while (true) {
      wake.wait(lock, [&] { return stopping || generation != observed_generation; });
      if (stopping) return;
      observed_generation = generation;
      const std::size_t count = task_count;
      const std::size_t active = active_workers;
      const Task current = task;
      lock.unlock();
      if (worker < active) {
        try {
          for (std::size_t index = worker; index < count; index += active) {
            current(index, worker, scratches[worker]);
          }
        } catch (...) {
          std::lock_guard<std::mutex> error_lock(mutex);
          if (!error) error = std::current_exception();
        }
      }
      lock.lock();
      if (--remaining == 0) finished.notify_one();
    }
  }

  std::vector<std::thread> threads;
  std::vector<RankUpdateScratch> scratches;
  std::mutex mutex;
  std::mutex invocation_mutex;
  std::condition_variable wake;
  std::condition_variable finished;
  Task task;
  std::size_t task_count = 0;
  std::size_t active_workers = 0;
  std::size_t remaining = 0;
  std::size_t generation = 0;
  bool stopping = false;
  std::exception_ptr error;
};

CandidateWorkerPool::CandidateWorkerPool(std::size_t count)
    : impl_(new Impl(count)) {}
CandidateWorkerPool::~CandidateWorkerPool() = default;
std::size_t CandidateWorkerPool::workerCount() const {
  return impl_->threads.size();
}

void CandidateWorkerPool::run(std::size_t task_count,
                              std::size_t active_workers,
                              const Task& task) {
  if (!task) throw std::invalid_argument("candidate worker task is empty");
  std::lock_guard<std::mutex> invocation(impl_->invocation_mutex);
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if (impl_->stopping) throw std::logic_error("candidate worker pool is stopped");
  impl_->task_count = task_count;
  impl_->active_workers = std::max<std::size_t>(
      1, std::min(active_workers, impl_->threads.size()));
  impl_->task = task;
  impl_->error = nullptr;
  impl_->remaining = impl_->threads.size();
  ++impl_->generation;
  impl_->wake.notify_all();
  impl_->finished.wait(lock, [&] { return impl_->remaining == 0; });
  const std::exception_ptr error = impl_->error;
  impl_->task = Task();
  lock.unlock();
  if (error) std::rethrow_exception(error);
}

void CandidateWorkerPool::clearScratch() {
  run(impl_->threads.size(), impl_->threads.size(),
      [](std::size_t, std::size_t, RankUpdateScratch& scratch) {
        scratch = RankUpdateScratch();
      });
}

}  // namespace uwb_imu_pl
