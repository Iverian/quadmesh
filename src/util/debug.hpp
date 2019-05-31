#ifndef QUADMESH_SRC_UTIL_DEBUG_HPP_
#define QUADMESH_SRC_UTIL_DEBUG_HPP_

#include <fmt/ostream.h>
#include <qmsh/debug.hpp>

#include <iostream>
#include <stdexcept>

#define cerrd                                                                 \
    if (!gm::debug_flag) {                                                    \
    } else                                                                    \
        std::cerr

#define coutd                                                                 \
    if (!gm::debug_flag) {                                                    \
    } else                                                                    \
        std::cout

#define throw_fmt(fmt_string, ...)                                            \
    do {                                                                      \
        auto what = fmt::format("({}:{}) " fmt_string, __FILE__, __LINE__,    \
                                ##__VA_ARGS__);                               \
        throw std::runtime_error(what);                                       \
    } while (0)

#define check_if(condition, fmt_string, ...)                                  \
    do {                                                                      \
        if (!bool(condition)) {                                               \
            throw_fmt(fmt_string, ##__VA_ARGS__);                             \
        }                                                                     \
    } while (0)

#ifdef NDEBUG

#define debug_fmt(stream, fmt_string, ...)
#define check_ifd(condition, fmt_string, ...)

#else // NDEBUG

#define debug_fmt(stream, fmt_string, ...)                                    \
    do {                                                                      \
        fmt::print((stream), fmt_string "\n", ##__VA_ARGS__);                 \
    } while (0)

#define check_ifd(condition, fmt_string, ...)                                 \
    check_if(condition, fmt_string, ##__VA_ARGS__)

#endif // NDEBUG

#endif // QUADMESH_SRC_UTIL_DEBUG_HPP_
