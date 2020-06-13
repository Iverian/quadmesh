#ifndef QUADMESH_SRC_MESH_CURVE_DISCRETIZE_HPP_
#define QUADMESH_SRC_MESH_CURVE_DISCRETIZE_HPP_

#include <gm/edge.hpp>
#include <qmsh/config.hpp>

#include <vector>

class CurveDiscretize {
    static constexpr size_t sample_size = 10;

public:
    explicit CurveDiscretize(const qmsh::Config& conf);
    virtual ~CurveDiscretize();
    virtual std::vector<double> param(const gm::AbstractCurve& curve,
                                      double pfront, double pback) const;

    std::vector<double> subdivide(const gm::AbstractCurve& curve,
                                  double pfront, double pback,
                                  double segment_length) const;

    std::vector<double> param(const gm::Edge& edge) const;
    std::vector<gm::Point> operator()(const gm::Edge& edge) const;
    std::vector<gm::Point> operator()(const gm::AbstractCurve& curve,
                                      double pfront, double pback) const;

protected:
    std::vector<double> curvature_discretize(const gm::AbstractCurve& curve,
                                             double pfront,
                                             double pback) const;
    double approx_length(const gm::AbstractCurve& curve,
                         const std::vector<double>& param) const;
    double curv_step(const gm::AbstractCurve& curve, double u) const;

private:
    size_t approx_len_mesh_size_;
    size_t div_curvature_coeff_;
    std::array<double, sample_size> samples_;
};

#endif // QUADMESH_SRC_MESH_CURVE_DISCRETIZE_HPP_
