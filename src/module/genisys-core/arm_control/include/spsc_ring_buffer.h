#pragma once

#include <atomic>
#include <cstddef>

template <typename T>
class SPSCRingBuffer {
 public:
  explicit SPSCRingBuffer(size_t capacity) 
    : capacity_(capacity), buffer_(new T[capacity_ + 1]) {}
  ~SPSCRingBuffer() { delete[] buffer_; }

  SPSCRingBuffer(const SPSCRingBuffer&) = delete;
  SPSCRingBuffer& operator=(const SPSCRingBuffer&) = delete;
  SPSCRingBuffer(SPSCRingBuffer&&) = delete;
  SPSCRingBuffer& operator=(SPSCRingBuffer&&) = delete;

  bool Push(const T &command) {
    if (IsFull()) return false;
    buffer_[tail_.load()] = command;
    tail_.store((tail_.load() + 1) % (capacity_ + 1));
    return true;
  }

  bool Pop(T &command) {
    if (IsEmpty()) return false;
    command = buffer_[head_.load()];
    head_.store((head_.load() + 1) % (capacity_ + 1));
    return true;
  }

  bool IsEmpty() const { return head_.load() == tail_.load(); }

  bool IsFull() const {
    return (tail_.load() + 1) % (capacity_ + 1) == head_.load();
  }

  size_t Size() const {
    size_t h = head_.load();
    size_t t = tail_.load();
    if (t >= h)
      return t - h;
    else
      return (capacity_ + 1) - h + t;
  }

  void Reset() {
    head_.store(0);
    tail_.store(0);
  }

 private:
  size_t capacity_;
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
  T *buffer_;
};