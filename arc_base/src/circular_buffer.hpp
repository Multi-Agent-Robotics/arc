#pragma once

#include <array>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

template <typename T, std::size_t N> class CircularBuffer {
  public:
    explicit CircularBuffer() : head(0), tail(0), count(0), sum(0.0) {}

    auto push_back(const T &value) -> void {
        if (count < N) {
            sum += value;
        } else {
            sum -= buffer[head];
            sum += value;
            head = (head + 1) % N;
        }

        buffer[tail] = value;
        tail = (tail + 1) % N;

        if (count < N) {
            ++count;
        }
    }

    auto operator[](std::size_t index) -> T & {
        if (index >= count) {
            throw std::out_of_range("Index out of range.");
        }
        return buffer[(head + index) % N];
    }

    auto operator[](std::size_t index) const -> const T & {
        if (index >= count) {
            throw std::out_of_range("Index out of range.");
        }
        return buffer[(head + index) % N];
    }

    auto size() const -> std::size_t { return count; }

    auto get_underlying_array() -> const std::array<T, N> & { return buffer; }

    auto mean() -> double {
        if (count == 0) {
            return 0.0;
        }
        return sum / static_cast<decltype(*buffer.begin() + 0.0)>(count);
    }

  private:
    std::array<T, N> buffer;
    std::size_t head;
    std::size_t tail;
    std::size_t count;
    double sum;
};
