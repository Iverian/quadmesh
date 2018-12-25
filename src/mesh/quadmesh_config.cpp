#define _USE_MATH_DEFINES

#include <quadmesh/quadmesh_config.h>

#include <cmath>

using namespace std;

static constexpr auto def_at = M_PI / 8;

struct QuadmeshConfig::Impl {
    Impl();
    explicit Impl(ifstream&& is);
    explicit Impl(const string& filename);

    const array<double, 6>& get_angle_tolerances() const;
    const pair<size_t, size_t> get_discretize_coeffs() const;

private:
    array<double, 6> angle_tolerances_;
    pair<size_t, size_t> discretize_coeffs_;
};

QuadmeshConfig::QuadmeshConfig()
    : pimpl_(make_unique<Impl>())
{
}

QuadmeshConfig::QuadmeshConfig(ifstream&& is)
    : pimpl_(make_unique<Impl>(move(is)))
{
}

QuadmeshConfig::QuadmeshConfig(const string& filename)
    : pimpl_(make_unique<Impl>(filename))
{
}

const array<double, 6>& QuadmeshConfig::get_angle_tolerances() const
{
    return pimpl_->get_angle_tolerances();
}

const pair<size_t, size_t> QuadmeshConfig::get_discretize_coeffs() const
{
    return pimpl_->get_discretize_coeffs();
}

QuadmeshConfig::Impl::Impl()
    : angle_tolerances_{def_at, def_at, def_at, def_at, def_at, def_at}
{
}

QuadmeshConfig::Impl::Impl(ifstream&& is)
    : angle_tolerances_()
{
}

QuadmeshConfig::Impl::Impl(const string& filename)
    : Impl(ifstream(filename))
{
}

const array<double, 6>& QuadmeshConfig::Impl::get_angle_tolerances() const
{
    return angle_tolerances_;
}

const pair<size_t, size_t> QuadmeshConfig::Impl::get_discretize_coeffs() const
{
    return {10, 10};
}