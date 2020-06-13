#ifndef QUADMESH_SRC_UTIL_TO_STRING_HPP_
#define QUADMESH_SRC_UTIL_TO_STRING_HPP_

#include <sstream>
#include <string>

using namespace std;

string to_string(bool x);

template <typename T>
string to_string(const T& t)
{
    std::stringstream ss;
    ss << t;
    return ss.str();
}

#endif // QUADMESH_SRC_UTIL_TO_STRING_HPP_
