#pragma once
#include "operator/grid_operator.hpp"

namespace VXZ {

class SmoothOperator : public GridOperator {
public:
    explicit SmoothOperator(int iterations = 1, float threshold = 0.0f);
    bool apply(VXZ::VoxelGrid& grid) const override;
    
private:
    // 成员变量根据原始定义补充
};

}
