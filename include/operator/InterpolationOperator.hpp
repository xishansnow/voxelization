#pragma once
#include "operator/grid_operator.hpp"
#include <eigen3/Eigen/Dense>
namespace VXZ {

class InterpolationOperator : public GridOperator {
public:
    explicit InterpolationOperator(const Eigen::Vector3f& position);
    bool apply(VXZ::VoxelGrid& grid) const override;
    float get_value() const;
private:
    // 成员变量根据原始定义补充
    float interpolated_value_;
};

}
