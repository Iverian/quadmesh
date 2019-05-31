#ifndef QUADMESH_INCLUDE_QMSH_INOUT_HPP_
#define QUADMESH_INCLUDE_QMSH_INOUT_HPP_

#include "config.hpp"
#include "exports.hpp"
#include "mesh.hpp"

#include <gm/shell.hpp>

#include <string>
#include <vector>

namespace qmsh {

QMSH_EXPORT Mesh build_mesh(const gm::Shell& shell, const qmsh::Config& conf);
QMSH_EXPORT void json_export(const std::vector<Mesh>& obj,
                             const std::string& filename);

} // namespace qmsh

#endif // QUADMESH_INCLUDE_QMSH_INOUT_HPP_