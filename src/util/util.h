#ifndef QUADMESH_SRC_UTIL_UTIL_H_
#define QUADMESH_SRC_UTIL_UTIL_H_

#include <array>
#include <cmath>
#include <ostream>
#include <vector>

template <class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    os << "[";
    for (auto it = std::cbegin(v); it != std::cend(v); ++it)
        os << *it << (std::next(it) == std::cend(v) ? "" : ", ");
    return os << "]";
}

template <class T, size_t N>
std::ostream& operator<<(std::ostream& os, const std::array<T, N>& v)
{
    os << "[";
    for (size_t i = 0; i < N; ++i)
        os << v[i] << (i + 1 == N ? "" : ", ");
    return os << "]";
};

#endif // QUADMESH_SRC_UTIL_UTIL_H_
