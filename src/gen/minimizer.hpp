#ifndef QUADMESH_SRC_GEN_MINIMIZER_HPP_
#define QUADMESH_SRC_GEN_MINIMIZER_HPP_

#include <gm/surf_point.hpp>

#include <array>
#include <functional>
#include <optional>

namespace qmsh {

using SurfDistFunc = std::function<double(gm::SurfPoint)>;

class SurfMinimizer {
public:
    static constexpr size_t max_iter = 100;

    SurfMinimizer(std::array<SurfDistFunc, 6>&& f,
                  std::pair<gm::SurfPoint, gm::SurfPoint> lim);

    std::optional<gm::SurfPoint> operator()(gm::SurfPoint init) const;

private:
    gm::SurfPoint next_step(gm::SurfPoint r, double du, double dv) const
        noexcept;
    gm::SurfPoint box_check(gm::SurfPoint r) const noexcept;

    std::array<SurfDistFunc, 6> f_;
    std::pair<gm::SurfPoint, gm::SurfPoint> lim_;
};

} // namespace qmsh

#endif // QUADMESH_SRC_GEN_MINIMIZER_HPP_