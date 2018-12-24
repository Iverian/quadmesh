#include "curve_discretize.h"

using namespace std;

CurveDiscretize::CurveDiscretize(const QuadmeshConfig& conf)
    : conf_(conf)
{
}

vector<double> CurveDiscretize::param(const Edge& edge) const
{
}

vector<double> CurveDiscretize::param(shared_ptr<AbstractCurve> curve,
                                      double pfront, double pback) const
{
}

vector<Point> CurveDiscretize::operator()(const Edge& edge) const
{
}

vector<Point> CurveDiscretize::operator()(shared_ptr<AbstractCurve> curve,
                                          double pfront, double pback) const
{
}