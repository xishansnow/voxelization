#pragma once
#include "operator/grid_operator.hpp"
#include <eigen3/Eigen/Dense>
namespace VXZ {

class FillOperator : public GridOperator {
public:
    explicit FillOperator(const Eigen::Vector3i& seed_point, int connectivity = 6);
    bool apply(VXZ::VoxelGrid& grid) const override;
    
private:
    // 成员变量根据原始定义补充
};

}
