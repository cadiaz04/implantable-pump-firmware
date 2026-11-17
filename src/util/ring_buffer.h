#pragma once
// Fixed-capacity ring buffer. Overwrites the oldest element when full.
// Not thread-safe on its own — callers guard it with the app mutex.
#include <cstddef>

template <typename T, size_t N>
class RingBuffer {
public:
    void push(const T& v) {
        buf_[head_] = v;
        head_ = (head_ + 1) % N;
        if (size_ < N) ++size_;
    }
    void clear() { head_ = 0; size_ = 0; }
    size_t size() const { return size_; }
    static constexpr size_t capacity() { return N; }
    bool empty() const { return size_ == 0; }
    // i = 0 is the oldest element.
    const T& at(size_t i) const { return buf_[(head_ + N - size_ + i) % N]; }
    const T& newest() const { return buf_[(head_ + N - 1) % N]; }

private:
    T buf_[N] = {};
    size_t head_ = 0;
    size_t size_ = 0;
};
