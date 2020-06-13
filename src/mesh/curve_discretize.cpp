#include "curve_discretize.hpp"

#include <gm/compare.hpp>
#include <gm/line.hpp>

#include <random>
#include <tuple>

CurveDiscretize::CurveDiscretize(const qmsh::Config& conf)
    : approx_len_mesh_size_()
    , div_curvature_coeff_()
    , samples_()
{
    std::tie(approx_len_mesh_size_, div_curvature_coeff_)
        = conf.discr_coeffs();

    auto step = 1. / (sample_size + 1);
    auto x = step;
    for (size_t i = 0; i < sample_size; ++i) {
        samples_[i] = x;
        x += step;
    }
    std::sort(std::begin(samples_), std::end(samples_));
}

CurveDiscretize::~CurveDiscretize() = default;

std::vector<double> CurveDiscretize::subdivide(const gm::AbstractCurve& curve,
                                               double pfront, double pback,
                                               double segment_length) const
{
    static constexpr auto d_eps = 1e-1;

    auto d_max = 0.;
    auto segment_coeff = 0.;
    auto t_max = pfront;
    auto c0 = curve(pfront);
    auto cd = curve(pback) - c0;

    for (auto& i : samples_) {
        auto t = pfront + i * (pback - pfront);
        auto m = curve(t);
        auto n = c0 + i * cd;
        auto d = gm::dist(m, n);
        if (gm::cmp::ge(d, d_max)) {
            segment_coeff = gm::dist(c0, m) / segment_length;
            d_max = d;
            t_max = t;
        }
    }

    if (gm::cmp::ge(d_max, d_eps)) {
        auto t_left = subdivide(curve, pfront, t_max, segment_length);
        auto t_right = subdivide(curve, t_max, pback, segment_length);
        t_left.reserve(t_right.size() - 1);
        std::copy(std::next(std::begin(t_right)), std::end(t_right),
                  std::back_inserter(t_left));
        return t_left;
    }
    return {pfront, t_max, pback};
}

std::vector<double> CurveDiscretize::param(const gm::AbstractCurve& curve,
                                           double pfront, double pback) const
{
    std::vector<double> result;

    auto reverse_flag = gm::cmp::le(pback, pfront);
    if (reverse_flag) {
        std::swap(pfront, pback);
    }

    result = curvature_discretize(curve, pfront, pback);
    static constexpr auto segment_length_mult = 3e-2;
    auto len = approx_length(curve, result);
    if (!gm::cmp::near(len, gm::dist(curve(pfront), curve(pback)))) {
        result = subdivide(curve, pfront, pback, segment_length_mult * len);
    }

    if (reverse_flag) {
        std::reverse(begin(result), end(result));
    }
    return result;
}

std::vector<double>
CurveDiscretize::curvature_discretize(const gm::AbstractCurve& curve,
                                      double pfront, double pback) const
{
    std::vector<double> result;

    auto cdelta = (pback - pfront) / div_curvature_coeff_;
    auto t = pfront;
    while (gm::cmp::le(t, pback)) {
        result.emplace_back(t);
        t += cdelta * curv_step(curve, t);
    }
    result.back() = pback;
    result.shrink_to_fit();

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

double CurveDiscretize::approx_length(const gm::AbstractCurve& curve,
                                      const std::vector<double>& params) const
{
    auto result = 0.;
    auto prev = 0.;
    auto cur = gm::norm(curve.df(params.front()));
    for (auto i = std::next(std::begin(params)); i != std::end(params); ++i) {
        prev = cur;
        cur = gm::norm(curve.df(*i));
        result += (*i - *std::prev(i)) * (prev + cur) / 2;
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
