#include <array>
#include <cstddef>

template <typename T, size_t Capacity>
class Ring {
    std::array<T, Capacity> buffer;
    size_t start;
    size_t end;
    size_t count;

   public:
    Ring() : start{0}, end{0}, count{0} {}

    size_t size() { return count; }

    size_t capacity() { return Capacity; }

    void push_back(T val) {
        buffer[end] = val;
        end = (end + 1) % capacity();

        if (full())
            start = end;
        else
            ++count;
    }

    void pop_front() {
        assert(!empty());
        start = (start + 1) % capacity();
        --count;
    }

    bool empty() { return size() == 0; }

    bool full() { return size() == capacity(); }

    T &front() {
        assert(!empty());
        return buffer[start];
    }

    T &back() {
        assert(!empty());
        return (*this)[count - 1];
    }

    T &operator[](size_t idx) {
        assert(idx < size());
        return buffer[(start + idx) % capacity()];
    }
};
