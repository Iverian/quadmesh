#ifndef QUADMESH_INLCUDE_QUADMESH_QUADMESH_CONFIG_H_
#define QUADMESH_INLCUDE_QUADMESH_QUADMESH_CONFIG_H_

#include <array>
#include <fstream>
#include <memory>

class QuadmeshConfig {
public:
    QuadmeshConfig();
    explicit QuadmeshConfig(std::ifstream&& is);
    explicit QuadmeshConfig(const std::string& filename);

    const std::array<double, 6>& get_angle_tolerances() const;
    const std::pair<size_t, size_t> get_discretize_coeffs() const;

private:
    struct Impl;
    std::shared_ptr<Impl> pimpl_;
};

#endif // QUADMESH_INLCUDE_QUADMESH_QUADMESH_CONFIG_H_