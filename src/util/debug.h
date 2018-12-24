#ifndef QUADMESH_SRC_UTIL_DEBUG_H_
#define QUADMESH_SRC_UTIL_DEBUG_H_

#include <exception>
#include <iostream>

#include <fmt/ostream.h>

#ifdef _MSC_VER
#define _WHAT_MODIFIERS
#elif defined(__GNUG__)
#define _WHAT_MODIFIERS _GLIBCXX_USE_NOEXCEPT
#else
#define _WHAT_MODIFIERS
#endif

#ifdef NDEBUG
#define DEBUG_FLAG false
#define dbg if (false)
#else
#define DEBUG_FLAG true
#define dbg if (true)
#endif

#define cdbg                                                                  \
    if (!DEBUG_FLAG) {                                                        \
    } else                                                                    \
        std::cerr

#define THROW_(exception, fmt_string, ...)                                    \
    do {                                                                      \
        auto what = fmt::format("({0}, {1}): ", __FILE__, __LINE__);          \
        what += fmt::format((fmt_string), __VA_ARGS__);                       \
        throw exception(what);                                                \
    } while (0)

#define CHECK_(condition, fmt_string, ...)                                    \
    do {                                                                      \
        if (!(condition)) {                                                   \
            THROW_(std::runtime_error, fmt_string, __VA_ARGS__);              \
        }                                                                     \
    } while (0)

#define DEBUG_FMT_(fmt_string, ...)                                           \
    do {                                                                      \
        if (DEBUG_FLAG) {                                                     \
            fmt::print(std::cout, (fmt_string), __VA_ARGS__);                 \
            std::cout << std::endl;                                           \
        }                                                                     \
    } while (0)

#endif // QUADMESH_SRC_UTIL_DEBUG_H_
