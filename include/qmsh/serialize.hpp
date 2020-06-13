#ifndef QUADMESH_INCLUDE_QMSH_SERIALIZE_HPP_
#define QUADMESH_INCLUDE_QMSH_SERIALIZE_HPP_

#include <ostream>
#include <type_traits>

#include <rapidjson/document.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>

namespace qmsh {

inline constexpr int serialize_max_decimal_places = 5;

namespace detail {
    template <class T>
    struct sfinae_true : std::true_type {
    };
    template <class T>
    inline constexpr bool sfinae_true_v = sfinae_true<T>::value;

    template <class T>
    static sfinae_true<decltype(&T::serialize)> test_serializable(int);
    template <class>
    static std::false_type test_serializable(long);
} // namespace detail

template <class T>
struct is_serializable : decltype(detail::test_serializable<T>(0)) {
};
template <class T>
inline constexpr bool is_serializable_v = is_serializable<T>::value;

template <class T, class = std::enable_if_t<is_serializable_v<T>>>
std::ostream& operator<<(std::ostream& os, const T& obj)
{
    rapidjson::Document d;
    auto& alloc = d.GetAllocator();
    obj.serialize(d, alloc);

    rapidjson::OStreamWrapper osw(os);
    rapidjson::Writer writer(osw);
    writer.SetMaxDecimalPlaces(serialize_max_decimal_places);
    d.Accept(writer);

    return os;
}

} // namespace qmsh

#endif // QUADMESH_INCLUDE_QMSH_SERIALIZE_HPP_
