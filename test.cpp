#include <cassert>
#include "Ring.hpp"

int main() {
    Ring<u_int32_t, 8> ring;
    assert(ring.empty() == true);
    assert(ring.full() == false);
    ring.push_back(0);
    ring.push_back(1);
    assert(ring[0] == 0);
    assert(ring.front() == 0);
    assert(ring[1] == 1);
    assert(ring.back() == 1);
    assert(ring.size() == 2);
    assert(ring.capacity() == 8);
    assert(ring.empty() == false);
    for (int i = 2; i < 8; ++i) {
        ring.push_back(i);
    }
    assert(ring.size() == 8);
    assert(ring.full() == true);
    assert(ring.back() == 7);
    assert(ring.front() == 0);
    ring.push_back(10);
    assert(ring.back() == 10);
    assert(ring.full());
    assert(ring.empty() == false);
    assert(ring.front() == 1);
    ring.pop_front();
    assert(ring.full() == false);
    assert(ring.front() == 2);
    assert(ring.back() == 10);
}

