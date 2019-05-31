#define _USE_MATH_DEFINES

#include <qmsh/config.hpp>

#include <cmath>

namespace qmsh {

static constexpr auto da = M_PI / 8;

Config::Config()
    : angtol_ {}
    , discr_coeffs_ {9, 9}
{
    angtol_.fill(da);
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