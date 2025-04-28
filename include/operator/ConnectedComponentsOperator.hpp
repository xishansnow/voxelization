#pragma once
#include "operator/grid_operator.hpp"

namespace VXZ {

class ConnectedComponentsOperator : public GridOperator {
public:
    explicit ConnectedComponentsOperator(int connectivity = 6);
    bool apply(VXZ::VoxelGrid& grid) const override;
    int get_component_count() const;
private:
    // 成员变量根据原始定义补充
};

}
