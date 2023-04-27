#pragma once

#include <array>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

template <typename T, std::size_t N> class CircularBuffer {
public:
	// Constructor: initializes the buffer with the specified size
	explicit CircularBuffer() : head(0), tail(0), count(0) {
	}

	// push_back: adds an element to the buffer, overwriting the oldest element if
	// the buffer is full
	auto push_back(const T &value) -> void {
		buffer[tail] = value;
		tail = (tail + 1) % N;

		// If the buffer isn't full, increment the count
		// Otherwise, increment head to maintain the correct start position
		if (count < N) {
			++count;
		} else {
			head = (head + 1) % N;
		}
	}

	// operator[]: provides access to elements in the buffer using an index
	auto operator[](std::size_t index) -> T & {
		if (index >= count) {
			throw std::out_of_range("Index out of range.");
		}
		return buffer[(head + index) % N];
	}

	// const operator[]: provides const access to elements in the buffer using an
	// index
	auto operator[](std::size_t index) const -> const T & {
		if (index >= count) {
			throw std::out_of_range("Index out of range.");
		}
		return buffer[(head + index) % N];
	}

	// size: returns the current number of elements in the buffer
	auto size() const -> std::size_t {
		return count;
	}

	auto get_underlying_array() -> const std::array<T, N> & {
		return buffer;
	}

	auto mean() -> double {
		if (count == 0) {
			return 0.0;
		}
		auto sum = std::accumulate(buffer.begin(), buffer.end(), 0.0);
		return sum / static_cast<decltype(*buffer.begin() + 0.0)>(count);
	}

private:
	std::array<T, N> buffer;
	std::size_t head;
	std::size_t tail;
	std::size_t count;
};
