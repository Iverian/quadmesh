#include "curve_discretize.hpp"

#include <gm/compare.hpp>
#include <gm/line.hpp>

#include <tuple>

CurveDiscretize::CurveDiscretize(const qmsh::Config& conf)
    : approx_len_mesh_size_()
    , div_curvature_coeff_()
{
    std::tie(approx_len_mesh_size_, div_curvature_coeff_)
        = conf.discr_coeffs();
}

CurveDiscretize::~CurveDiscretize() = default;

std::vector<double> CurveDiscretize::param(const gm::AbstractCurve& curve,
                                           double pfront, double pback) const
{
    std::vector<double> result;
    auto reverse_flag = gm::cmp::le(pback, pfront);
    if (reverse_flag) {
        std::swap(pfront, pback);
    }

    auto cdelta = (pback - pfront) / div_curvature_coeff_;
    auto t = pfront;
    while (gm::cmp::le(t, pback)) {
        result.emplace_back(t);
        t += cdelta * curv_step(curve, t);
    }
    result.back() = pback;
    result.shrink_to_fit();

    if (reverse_flag) {
        std::reverse(begin(result), end(result));
    }

    return result;
}

double CurveDiscretize::curv_step(const gm::AbstractCurve& curve,
                                  double u) const
{
    double result = 1;
    if (auto d2 = curve.df2(u); !gm::cmp::zero(d2)) {
        auto d1 = curve.df(u);
        result = sqr(d1) / norm(cross(d1, d2));
    }
    return result;
}

std::vector<double> CurveDiscretize::param(const gm::Edge& edge) const
{
    return param(*edge.curve(), edge.pfront(), edge.pback());
}

std::vector<gm::Point> CurveDiscretize::operator()(const gm::Edge& edge) const
{
    return this->operator()(*edge.curve(), edge.pfront(), edge.pback());
}

std::vector<gm::Point>
CurveDiscretize::operator()(const gm::AbstractCurve& curve, double pfront,
                            double pback) const
{
    auto v = param(curve, pfront, pback);
    std::vector<gm::Point> result(v.size());
    std::transform(begin(v), end(v), begin(result),
                   [&curve](const auto& t) { return curve.f(t); });
    return result;
}
