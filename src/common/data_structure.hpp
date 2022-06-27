#pragma once
#include "common.hpp"
#include <array>
#include <atomic>

#include <mutex>

namespace common {
// queue
template <typename T> class Queue {
private:
  // mutable std::mutex mutex_;
  // Condition Variable
  // std::condition_variable cond_;
  // current queue size
  int current_size_ = 0;
  // int current_size_ = 0;

  // queue_length
  int max_size_ = 0;

  // front pointer
  int front_ptr_ = 0;

  // back pointer
  int back_ptr_ = 0;

  // queue pointer
  T *q = nullptr;

public:
  Queue() = default;
  Queue(const Queue<T> &) = delete;
  Queue(Queue<T> &&other) = delete;
  Queue &operator=(const Queue<T> &other) = delete;
  Queue &operator=(Queue<T> &&other) = delete;

  ~Queue() {
    // free memory
    delete[] q;
    current_size_ = 0;
  }

  // init Queue with known type and queue length
  Queue(const int &q_length) : max_size_(q_length) {
    q = new T[max_size_];
    if (q == NULL) {
      perror("\rqueue malloc memory failed!\n");
    }
  }

  // Create Queue with length
  void Create(const int &q_length) {
    max_size_ = q_length;
    q = new T[max_size_];
    if (q == NULL) {
      perror("\rqueue malloc memory failed!\n");
    }
  }

  inline bool QueueISFull(void) const {
    if (current_size_ == max_size_)
      return true;
    else
      return false;
  }

  inline bool QueueISEmpty(void) const {
    if (current_size_ == 0)
      return true;
    else
      return false;
  }

  inline bool Empty(void) const { return QueueISEmpty(); }

  // enqueue with reference
  void EnQueue(const T &item) {
    // std::lock_guard<std::mutex> lock(mutex_);
    // not initial state
    if (current_size_ != 0) {
      // point to the end
      if (back_ptr_ == max_size_ - 1)
        // point to the front
        back_ptr_ = 0;
      else
        back_ptr_++;
    }
    q[back_ptr_] = item;
    // increase current size
    current_size_++;

    // cond_.notify_all();
  }

  // enqueue with r value
  void EnQueue(const T &&item) {
    // std::lock_guard<std::mutex> lock(mutex_);
    // not initial state
    if (current_size_ != 0) {
      // point to the end
      if (back_ptr_ == max_size_ - 1)
        // point to the front
        back_ptr_ = 0;
      else
        back_ptr_++;
    }
    q[back_ptr_] = item;
    // increase current size
    current_size_++;

    // cond_.notify_all();
  }

  // dequeque
  T *DeQueue(void) {
    // std::lock_guard<std::mutex> lock(mutex_);
    // cond_.wait(lock, []() { return !QueueISEmpty(); });
    if (current_size_ > 0) {
      // decrease current size
      current_size_--;

      // point to the end
      if (front_ptr_ == max_size_ - 1) { // point to the front
        front_ptr_ = 0;
        return &q[max_size_ - 1];
      } else {
        front_ptr_++;
        return &q[front_ptr_ - 1];
      }
    }
  }

  // front pointer
  T *Front() const { return &q[front_ptr_]; }

  // back pointer
  T *Back(void) const { return &q[back_ptr_]; }

  // return current size
  int Size(void) const { return current_size_; }
};
} // namespace common