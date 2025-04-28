#include "operator/InterpolationOperator.hpp"
#include <eigen3/Eigen/Dense>
namespace VXZ {

InterpolationOperator::InterpolationOperator(const Eigen::Vector3f& position) {}

bool InterpolationOperator::apply(VXZ::VoxelGrid& grid) const {
    // TODO: 实现 InterpolationOperator 算法
    return true;
}
float InterpolationOperator::get_value() const { return interpolated_value_; }
}
