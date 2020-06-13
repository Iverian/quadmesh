#ifndef QUADMESH_INCLUDE_QMSH_DEBUG_HPP_
#define QUADMESH_INCLUDE_QMSH_DEBUG_HPP_

namespace qmsh {

#ifdef NDEBUG

#define __QMSH_NOEXCEPT_RELEASE__ noexcept
static constexpr auto debug_flag = false;

#else // NDEBUG

#define __QMSH_NOEXCEPT_RELEASE__
static constexpr auto debug_flag = true;

#endif // NDEBUG

} // namespace qmsh

#endif // QUADMESH_INCLUDE_QMSH_DEBUG_HPP_
