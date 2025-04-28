#pragma once
#include "operator/grid_operator.hpp"

namespace VXZ {

class DifferenceOperator : public GridOperator {
public:
    explicit DifferenceOperator(const VXZ::VoxelGrid& other);
    bool apply(VXZ::VoxelGrid& grid) const override;
    
private:
    // 成员变量根据原始定义补充
};

}
