#ifndef QUADMESH_INCLUDE_QMSH_CONFIG_HPP_
#define QUADMESH_INCLUDE_QMSH_CONFIG_HPP_

#include "exports.hpp"
#include "serialize.hpp"

#include <array>
#include <fstream>
#include <memory>
#include <vector>

namespace qmsh {

class QMSH_EXPORT Config {
public:
    using AngTol = std::array<double, 6>;
    using DiscrCoeffs = std::pair<size_t, size_t>;

    Config();

    const AngTol& angtol() const noexcept;
    const DiscrCoeffs& discr_coeffs() const noexcept;

    AngTol& angtol() noexcept;
    DiscrCoeffs& discr_coeffs() noexcept;

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;

private:
    AngTol angtol_;
    DiscrCoeffs discr_coeffs_;
};

} // namespace qmsh

#endif // QUADMESH_INCLUDE_QMSH_CONFIG_HPP_