#pragma once
#include "operator/grid_operator.hpp"

namespace VXZ {

class IntersectionOperator : public GridOperator {
public:
    explicit IntersectionOperator(const VXZ::VoxelGrid& other);
    bool apply(VXZ::VoxelGrid& grid) const override;
    
private:
    // 成员变量根据原始定义补充
};

}
