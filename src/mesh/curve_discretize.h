#ifndef QUADMESH_SRC_MESH_CURVE_DISCRETIZE_H_
#define QUADMESH_SRC_MESH_CURVE_DISCRETIZE_H_

#include <gm/edge.hpp>
#include <quadmesh/quadmesh_config.h>

#include <vector>

class CurveDiscretize {
public:
    explicit CurveDiscretize(const QuadmeshConfig& conf);
    virtual ~CurveDiscretize();
    virtual std::vector<double> param(const gm::AbstractCurve& curve,
                                      double pfront, double pback) const;

    std::vector<double> param(const gm::Edge& edge) const;
    std::vector<gm::Point> operator()(const gm::Edge& edge) const;
    std::vector<gm::Point> operator()(const gm::AbstractCurve& curve, double pfront,
                                  double pback) const;

protected:
    double curv_step(const gm::AbstractCurve& curve, double u) const;

private:
    size_t approx_len_mesh_size_;
    size_t div_curvature_coeff_;
};

#endif