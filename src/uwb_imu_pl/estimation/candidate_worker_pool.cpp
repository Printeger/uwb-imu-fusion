#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace uwb_imu_pl {

namespace {

thread_local const void* active_pool = nullptr;

std::size_t matrixBytes(const Eigen::MatrixXd& value) {
  return static_cast<std::size_t>(value.size()) * sizeof(double);
}

std::size_t vectorBytes(const Eigen::VectorXd& value) {
  return static_cast<std::size_t>(value.size()) * sizeof(double);
}

std::size_t scratchBytes(const RankUpdateScratch& scratch) {
  return matrixBytes(scratch.update_columns) +
      vectorBytes(scratch.update_signs) +
      matrixBytes(scratch.whitened_update) +
      matrixBytes(scratch.small_symmetric);
}

}  // namespace

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
      const bool flat = flat_scheduling;
      const std::size_t scratch_limit = scratch_limit_bytes;
      const Task current = task;
      lock.unlock();
      if (worker < active) {
        const void* prior_pool = active_pool;
        active_pool = this;
        if (flat) {
          while (true) {
            const std::size_t index = next_task.fetch_add(1);
            if (index >= count) break;
            std::exception_ptr task_error;
            try {
              current(index, worker, scratches[worker]);
            } catch (...) {
              task_error = std::current_exception();
            }
            const std::size_t bytes =
                uwb_imu_pl::scratchBytes(scratches[worker]);
            std::size_t observed = scratch_high_water_bytes.load();
            while (observed < bytes &&
                   !scratch_high_water_bytes.compare_exchange_weak(
                       observed, bytes)) {}
            const bool scratch_exceeded =
                scratch_limit > 0 && bytes > scratch_limit;
            if (scratch_exceeded) {
              // The allocation has already occurred. Drop the oversized
              // retained storage immediately; scheduling still drives every
              // task to the common barrier before this identity fails closed.
              scratches[worker] = RankUpdateScratch();
            }
            if (task_error || scratch_exceeded) {
              std::exception_ptr selected = task_error;
              if (!selected) {
                selected = std::make_exception_ptr(std::runtime_error(
                    "candidate worker scratch limit exceeded at task " +
                    std::to_string(index)));
              }
              std::lock_guard<std::mutex> error_lock(mutex);
              if (index < error_index) {
                error_index = index;
                error = std::move(selected);
              }
            }
          }
        } else {
          try {
            for (std::size_t index = worker; index < count; index += active) {
              current(index, worker, scratches[worker]);
            }
          } catch (...) {
            std::lock_guard<std::mutex> error_lock(mutex);
            if (!error) error = std::current_exception();
          }
          const std::size_t bytes =
              uwb_imu_pl::scratchBytes(scratches[worker]);
          std::size_t observed = scratch_high_water_bytes.load();
          while (observed < bytes &&
                 !scratch_high_water_bytes.compare_exchange_weak(
                     observed, bytes)) {}
        }
        active_pool = prior_pool;
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
  std::atomic<std::size_t> next_task{0};
  std::size_t error_index = std::numeric_limits<std::size_t>::max();
  std::size_t scratch_limit_bytes = 0;
  std::atomic<std::size_t> scratch_high_water_bytes{0};
  bool flat_scheduling = false;
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
  if (active_pool == impl_.get()) {
    throw std::logic_error("nested candidate worker pool invocation");
  }
  std::lock_guard<std::mutex> invocation(impl_->invocation_mutex);
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if (impl_->stopping) throw std::logic_error("candidate worker pool is stopped");
  impl_->task_count = task_count;
  impl_->active_workers = std::max<std::size_t>(
      1, std::min(active_workers, impl_->threads.size()));
  impl_->task = task;
  impl_->flat_scheduling = false;
  impl_->scratch_limit_bytes = 0;
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

void CandidateWorkerPool::runFlat(std::size_t task_count,
                                  std::size_t active_workers,
                                  std::size_t scratch_limit_bytes,
                                  const Task& task) {
  if (!task) throw std::invalid_argument("candidate worker task is empty");
  if (active_pool == impl_.get()) {
    throw std::logic_error("nested candidate worker pool invocation");
  }
  std::lock_guard<std::mutex> invocation(impl_->invocation_mutex);
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if (impl_->stopping) throw std::logic_error("candidate worker pool is stopped");
  impl_->task_count = task_count;
  impl_->active_workers = std::max<std::size_t>(
      1, std::min(active_workers, impl_->threads.size()));
  impl_->task = task;
  impl_->flat_scheduling = true;
  impl_->scratch_limit_bytes = scratch_limit_bytes;
  impl_->next_task = 0;
  impl_->error = nullptr;
  impl_->error_index = std::numeric_limits<std::size_t>::max();
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

std::size_t CandidateWorkerPool::scratchBytes() const {
  std::lock_guard<std::mutex> invocation(impl_->invocation_mutex);
  std::size_t bytes = 0;
  for (const auto& scratch : impl_->scratches) {
    bytes += uwb_imu_pl::scratchBytes(scratch);
  }
  return bytes;
}

std::size_t CandidateWorkerPool::scratchHighWaterBytes() const {
  std::lock_guard<std::mutex> invocation(impl_->invocation_mutex);
  return impl_->scratch_high_water_bytes.load();
}

}  // namespace uwb_imu_pl
