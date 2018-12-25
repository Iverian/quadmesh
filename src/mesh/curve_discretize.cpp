#include "curve_discretize.h"
#include <geom_model/geom_util.h>
#include <geom_model/line.h>

using namespace std;

CurveDiscretize::CurveDiscretize(const QuadmeshConfig& conf)
    : approx_len_mesh_size_()
    , div_curvature_coeff_()
{
    tie(approx_len_mesh_size_, div_curvature_coeff_)
        = conf.get_discretize_coeffs();
}

CurveDiscretize::~CurveDiscretize() = default;

vector<double> CurveDiscretize::param(const AbstractCurve& curve,
                                      double pfront, double pback) const
{
    vector<double> result;
    bool reverse_flag = pback <= pfront;
    if (reverse_flag) {
        swap(pfront, pback);
    }
    auto cdelta = (pback - pfront) / 6;

    result.emplace_back(pfront);
    while (result.back() < pback) {
        auto& t = result.back();
        auto dt = cdelta * curv_step(curve, t);
        result.emplace_back(t + dt);
    }
    result.back() = pback;
    result.shrink_to_fit();

    if (reverse_flag) {
        reverse(begin(result), end(result));
    }

    return result;
}

double CurveDiscretize::curv_step(const AbstractCurve& curve, double u) const
{
    double result = 1;
    if (auto d2 = curve.df2(u); !isnear(norm(d2), 0, 1e-5)) {
        auto d1 = curve.df(u);
        result = sqr(d1) / norm(cross(d1, d2));
    }
    return result;
}

vector<double> CurveDiscretize::param(const Edge& edge) const
{
    return param(edge.curve(), edge.pfront(), edge.pback());
}

vector<Point> CurveDiscretize::operator()(const Edge& edge) const
{
    return this->operator()(edge.curve(), edge.pfront(), edge.pback());
}

vector<Point> CurveDiscretize::operator()(const AbstractCurve& curve,
                                          double pfront, double pback) const
{
    auto v = param(curve, pfront, pback);
    vector<Point> result(v.size());
    transform(begin(v), end(v), begin(result),
              [&curve](const auto& t) { return curve.f(t); });
    return result;
}