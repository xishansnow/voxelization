#pragma once
#include "operator/grid_operator.hpp"
#include <limits>
namespace VXZ {

class DistanceTransformOperator : public GridOperator {
public:
    explicit DistanceTransformOperator(float max_distance = std::numeric_limits<float>::max());
    bool apply(VXZ::VoxelGrid& grid) const override;
    
private:
    // 成员变量根据原始定义补充
};

}
