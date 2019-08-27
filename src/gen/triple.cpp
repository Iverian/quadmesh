#define _USE_MATH_DEFINES
#include "triple.hpp"

#include <cmms/debug.hpp>

#include <cmath>

namespace qmsh {

double Triple::iangle(FrontType t) const
{
    auto a = gm::Vec(p[1], p[0]);
    auto b = gm::Vec(p[1], p[2]);

    auto result = gm::angle(a, b);
    auto d = gm::dot(gm::cross(a, b), norm);
    if (std::signbit(d) != bool(t)) {
        result = 2 * M_PI - result;
    }
    check_if(!std::isnan(result), "internal angle is nan / {}", *this);

    return result;
}

gm::Point& Triple::operator[](size_t i) noexcept
{
    return p[i];
}

const gm::Point& Triple::operator[](size_t i) const noexcept
{
    return p[i];
}

rapidjson::Value&
Triple::serialize(rapidjson::Value& result,
                  rapidjson::Value::AllocatorType& alloc) const
{
    rapidjson::Value vtxs(rapidjson::kArrayType);
    rapidjson::Value m(rapidjson::kArrayType);

    result.SetObject();
    for (auto& i : p) {
        rapidjson::Value point(rapidjson::kArrayType);
        for (auto& j : i) {
            point.PushBack(j, alloc);
        }
        vtxs.PushBack(point, alloc);
    }
    for (auto& i : norm) {
        m.PushBack(i, alloc);
    }
    result.AddMember("vertices", vtxs, alloc);
    result.AddMember("norm", m, alloc);

    return result;
}

} // namespace qmsh
