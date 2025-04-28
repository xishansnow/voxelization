#pragma once
#include "operator/grid_operator.hpp"

namespace VXZ {

class ClosingOperator : public GridOperator {
public:
    explicit ClosingOperator(int iterations = 1);
    bool apply(VXZ::VoxelGrid& grid) const override;
    
private:
    // 成员变量根据原始定义补充
};

}
