#ifndef QUADMESH_SRC_GEN_TRIPLE_HPP_
#define QUADMESH_SRC_GEN_TRIPLE_HPP_

#include "generation_front.hpp"

#include <gm/point.hpp>
#include <gm/vec.hpp>
#include <qmsh/mesh.hpp>
#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>

#include <array>

namespace qmsh {
struct Triple {
    std::array<gm::Point, 3> p;
    gm::Vec norm;

    double iangle(FrontType t) const;
    gm::Point& operator[](size_t i) noexcept;
    const gm::Point& operator[](size_t i) const noexcept;

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;
};
}

#endif // QUADMESH_SRC_GEN_TRIPLE_HPP_
