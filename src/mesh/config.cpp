#define _USE_MATH_DEFINES

#include <qmsh/config.hpp>

#include <cmath>

namespace qmsh {

Config::Config()
    : angtol_ {M_PI / 12, M_PI / 12, M_PI / 12, M_PI / 12, M_PI / 12, M_PI / 4}
    , discr_coeffs_ {10, 10}
{
}

const Config::AngTol& Config::angtol() const noexcept
{
    return angtol_;
}

const Config::DiscrCoeffs& Config::discr_coeffs() const noexcept
{
    return discr_coeffs_;
}

Config::AngTol& Config::angtol() noexcept
{
    return angtol_;
}

Config::DiscrCoeffs& Config::discr_coeffs() noexcept
{
    return discr_coeffs_;
}

rapidjson::Value&
Config::serialize(rapidjson::Value& result,
                  rapidjson::Value::AllocatorType& alloc) const
{
    rapidjson::Value angtol(rapidjson::kArrayType);
    rapidjson::Value coeffs(rapidjson::kArrayType);

    result.SetObject();
    for (auto& i : angtol_) {
        angtol.PushBack(i, alloc);
    }
    coeffs.PushBack(discr_coeffs_.first, alloc);
    coeffs.PushBack(discr_coeffs_.second, alloc);
    result.AddMember("angle_tolerances", angtol, alloc);
    result.AddMember("discretize_coeffs", coeffs, alloc);

    return result;
}

} // namespace qmsh
