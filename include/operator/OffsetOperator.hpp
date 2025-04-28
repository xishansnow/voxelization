#pragma once
#include "operator/grid_operator.hpp"

namespace VXZ {

class OffsetOperator : public GridOperator {
public:
    explicit OffsetOperator(float distance = 1.0f);
    bool apply(VXZ::VoxelGrid& grid) const override;
    
private:
    // 成员变量根据原始定义补充
};

}
