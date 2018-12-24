#ifndef QUADMESH_SRC_MESH_CURVE_DISCRETIZE_H_
#define QUADMESH_SRC_MESH_CURVE_DISCRETIZE_H_

#include <geom_model/edge.h>
#include <quadmesh/quadmesh_config.h>

#include <vector>

class CurveDiscretize {
    explicit CurveDiscretize(const QuadmeshConfig& conf);
    virtual ~CurveDiscretize();
    virtual std::vector<double> param(std::shared_ptr<AbstractCurve> curve,
                                      double pfront, double pback) const;

    std::vector<double> param(const Edge& edge) const;
    std::vector<Point> operator()(const Edge& edge) const;
    std::vector<Point> operator()(std::shared_ptr<AbstractCurve> curve,
                                  double pfront, double pback) const;

private:
    size_t approx_len_mesh_size_;
    size_t div_curvature_coeff_;
};

#endif