#pragma once

#include <cstdint>

// 仅管理帧提交状态；并发调用由外部临界区保护，像素读写由帧缓冲信号量保护。
class epdiy_async_flush_state {
 public:
  bool request_next_frame() {
    if (next_frame_requested_ || frame_active_ || submission_pending_) {
      return false;
    }
    next_frame_requested_ = true;
    return true;
  }

  // 同一帧可能分多次回调复制像素，只有最后一个分块才能进入待提交状态。
  bool begin_flush(bool is_last) {
    if (!frame_active_) {
      frame_active_ = true;
      // 所有帧均使用工作任务。上一物理刷新可能仍在进行，这里仅登记下一帧状态；
      // 调用方在复制新像素前仍须等待单帧缓冲信号量。
      frame_async_          = true;
      next_frame_requested_ = false;
    }

    bool async = frame_async_;
    if (is_last) {
      frame_active_ = false;
      frame_async_  = false;
      if (async) {
        submission_pending_ = true;
      }
    }
    return async;
  }

  // 同时只跟踪一个物理任务；前一任务完成后才能接受下一次有效提交。
  bool submit_job(uint64_t job_id) {
    if (!submission_pending_ || job_in_flight_ || job_id == 0) {
      return false;
    }
    submission_pending_ = false;
    job_in_flight_      = true;
    current_job_id_     = job_id;
    return true;
  }

  bool cancel_submission() {
    if (!submission_pending_ || job_in_flight_) {
      return false;
    }
    submission_pending_ = false;
    return true;
  }

  // 完成事件必须匹配当前任务，拒绝迟到或错误的任务编号。
  bool complete_job(uint64_t job_id) {
    if (!job_in_flight_ || current_job_id_ != job_id) {
      return false;
    }
    job_in_flight_  = false;
    current_job_id_ = 0;
    return true;
  }

  bool job_in_flight() const {
    return job_in_flight_;
  }

  uint64_t current_job_id() const {
    return current_job_id_;
  }

 private:
  bool     next_frame_requested_ = false;
  bool     frame_active_         = false;
  bool     frame_async_          = false;
  bool     submission_pending_   = false;
  bool     job_in_flight_        = false;
  uint64_t current_job_id_       = 0;
};
