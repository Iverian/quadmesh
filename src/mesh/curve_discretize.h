#ifndef QUADMESH_SRC_MESH_CURVE_DISCRETIZE_H_
#define QUADMESH_SRC_MESH_CURVE_DISCRETIZE_H_

#include <geom_model/edge.h>
#include <quadmesh/quadmesh_config.h>

#include <vector>

class CurveDiscretize {
public:
    explicit CurveDiscretize(const QuadmeshConfig& conf);
    virtual ~CurveDiscretize();
    virtual std::vector<double> param(const AbstractCurve& curve,
                                      double pfront, double pback) const;

    std::vector<double> param(const Edge& edge) const;
    std::vector<Point> operator()(const Edge& edge) const;
    std::vector<Point> operator()(const AbstractCurve& curve, double pfront,
                                  double pback) const;

protected:
    double curv_step(const AbstractCurve& curve, double u) const;

private:
    size_t approx_len_mesh_size_;
    size_t div_curvature_coeff_;
};

#endif