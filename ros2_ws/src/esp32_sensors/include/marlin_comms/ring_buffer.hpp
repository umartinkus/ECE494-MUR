#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include <utility>
#include <mutex>
#include <span>

template<typename T>
class Ring {
public:
    explicit Ring(std::size_t capacity)
    : buf_(capacity), head_(0), tail_(0), size_(0) {}

    std::size_t size() const noexcept { return size_; }
    std::size_t capacity() const noexcept { return buf_.size(); }
    bool empty() const noexcept { return size_ == 0; }
    bool full() const noexcept { return size_ == capacity(); }
    void clear() noexcept { head_ = 0; tail_ = 0; size_ = 0; }

    std::mutex get_mutex() { return mutex_; }

    // lvalue reference functions are for small amounts of data
    bool push_drop(const T& val) {
        std::lock_guard<std::mutex> guard(mutex_);

        if ( full() ) return false;
        buf_[head_] = val;
        head_ = inc_(head_);
        size_++;
        return true;
    }

    // overloaded rvalue reference functions are used for large amounts of data
    // should be more efficient for data ring isntead of UART ring
    bool push_drop(T&& val) {
        std::lock_guard<std::mutex> guard(mutex_);
        if ( full() ) return false;
        buf_[head_] = std::move(val);
        head_ = inc_(head_);
        size_++;
        return true;
    }

    void push_overwrite(const T& val) {
        std::lock_guard<std::mutex> guard(mutex_);
        buf_[head_] = val;
        head_ = inc_(head_);
        if ( full() ) {
            tail_ = inc_(tail_);
        } else {
            size_++;
        }
    }

    void push_overwrite(T&& val) {
        std::lock_guard<std::mutex> guard(mutex_);
        buf_[head_] = std::move(val);
        head_ = inc_(head_);
        if ( full() ) {
            tail_ = inc_(tail_);
        } else {
            size_++;
        }
    }

    bool pop(T& out) {
        if (empty()) return false;
        std::lock_guard<std::mutex> guard(mutex_);
        
        out = std::move(buf_[tail_]);
        tail_ = inc_(tail_);
        size--;
        return true;
    }

    const T* peek_oldest() const noexcept { 
        return empty() ? nullptr : &buf_[tail_];
    }
    const T* peek_newest() const noexcept {
        return empty() ? nullptr : &buf_[head_];
    }
    const T& peek_offset(std::size_t offset) const noexcept {
        return empty() ? nullptr : buf_[phys_index_(offset)];
    }

private:
    std::size_t inc_(std::size_t i) {
        i++;
        if ( i == capacity() ) i = 0;
        return i;
    }

    std::size_t phys_index_(std::size_t offset) {
        std::size_t idx = head_ + offset;
        const auto cap = capacity();
        if (idx >= capacity()) idx -= cap;
        return idx;
    }
    
    std::mutex mutex_;
    std::vector<T> buf_;
    std::size_t head_;
    std::size_t tail_;
    std::size_t size_;
};

// this is where the ByteRing Wrapper starts
class ByteRing {
public:
    explicit ByteRing(std::size_t capacity) : ring_(capacity) {}

    std::size_t size() const noexcept { return ring_.size(); }
    std::size_t capacity() const noexcept { return ring_.capacity(); }
    void clear() noexcept { ring_.clear(); }

    std::size_t write(std::span<const std::uint8_t> in) {
        std::lock_guard<std::mutex> lock(ring_.get_mutex());

        std::size_t n_written = 0;
        for(std::uint8_t pnt : in) {
            if( !ring_.push_drop(pnt) ) break;
            n_written++;
        }
        return n_written;
    }

    // pass in a span object with a given size and read that many bytes from the buffer
    std::size_t read(std::span<std::uint8_t> out) {
        std::lock_guard<std::mutex> lock(ring_.get_mutex());

        std::size_t n = 0;
        for (; n < out.size(); n++) {
            std::uint8_t b{};
            if ( !ring_.pop(b) ) break;
            out[n] = b;
        }
        return n;
    }

    std::size_t discard(std::size_t n) {
        std::lock_guard<std::mutex> lock(ring_.get_mutex());

        std::uint8_t b{};
        for (int i = 0; i < n; i++) {
            if ( !ring_.pop(b) ) return i;
        }
    }

    std::size_t peek(std::span<std::uint8_t> out) const {
        std::lock_guard<std::mutex> lock(ring_.get_mutex());
        const auto avail = ring.size();
        const auto n = std::min(avail, out.size());
        for (std::size_t i = 0; i < n; ++i) {
            out[i] = ring.peek_offset(i)
        }
        return n;
    }

    // returns max value of static_cast if there is no match
    static constexpr std::size_t no_val = static_cast<std::size_t>(-1);
    std::size_t find_pattern(std::span<std::uint8_t> pattern) {
        if ( pattern.size() > ring_.size() ) return no_val;

        const std::size_t limit = ring_.size() - pattern.size();
        for ( std::size_t i = 0; i <= limit; i++ ) {
            bool match = true;
            for ( std::size_t j = 0; j <= pattern.size(); j++ ) {
                if ( ring_.peek_offset(i + j) != pattern[j] ) {
                    match = false;
                    break;
                }
            }
            if (match) return i;
        }
        return no_val;
    }

    bool copy_window(size_t offset, std::span<uint8_t> out) const {
        if ( out.size() > ring_.size() ) return false;
        for (size_t i = 0; i < out.size(); i++){
          out[i] = ring.peek_offset(offset + i);
        }
        return true;
    }
private:
    Ring<std::uint8_t> ring_;
};
