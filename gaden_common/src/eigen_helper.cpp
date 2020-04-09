#include <gaden_common/eigen_helper.hpp>

#include <yaml-cpp/yaml.h>

namespace gaden::eigen_helper {

Eigen::Vector3d getVector3dFromYaml(const YAML::Node &sequence)
{
    return Eigen::Vector3d(sequence[0].as<double>(),
                           sequence[1].as<double>(),
                           sequence[2].as<double>());
}

} // namespace gaden::eigen_helper
