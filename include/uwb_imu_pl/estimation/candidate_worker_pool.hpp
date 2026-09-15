#pragma once

#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"

#include <cstddef>
#include <functional>
#include <memory>

namespace uwb_imu_pl {

// Persistent, deterministic candidate scheduler.  Tasks are assigned by
// index modulo active_workers; completion order can never affect output slots.
class CandidateWorkerPool {
 public:
  using Task = std::function<void(std::size_t task_index,
                                  std::size_t worker_index,
                                  RankUpdateScratch& scratch)>;

  explicit CandidateWorkerPool(std::size_t worker_count = 4);
  ~CandidateWorkerPool();
  CandidateWorkerPool(const CandidateWorkerPool&) = delete;
  CandidateWorkerPool& operator=(const CandidateWorkerPool&) = delete;

  std::size_t workerCount() const;
  void run(std::size_t task_count, std::size_t active_workers,
           const Task& task);
  void clearScratch();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uwb_imu_pl
