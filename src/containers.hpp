#pragma once
#include <cstddef>
#include <utility>
#include <cassert>

template<typename T, std::size_t N>
class InplaceVector {
private:
    alignas(T) std::byte data_[N * sizeof(T)];
    std::size_t size_ = 0;
public:
    constexpr InplaceVector() noexcept = default;
    constexpr InplaceVector(auto&&... args) noexcept : InplaceVector() {
        (emplace_back(std::forward<decltype(args)>(args)), ...);
    }
    constexpr void push_back(const T& value) noexcept {
        assert(size_ < N);
        new (&data_[size_ * sizeof(T)]) T(value);
        ++size_;
    }
    constexpr void emplace_back(T&& value) noexcept {
        assert(size_ < N);
        new (&data_[size_ * sizeof(T)]) T(std::move(value));
        ++size_;
    }
    constexpr std::size_t size() const noexcept { return size_; }
    constexpr T& operator[](std::size_t index) noexcept { return *reinterpret_cast<T*>(&data_[index * sizeof(T)]); }
    constexpr const T& operator[](std::size_t index) const noexcept { return *reinterpret_cast<const T*>(&data_[index * sizeof(T)]); }
};

