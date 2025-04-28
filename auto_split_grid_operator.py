import os
import re

# 路径配置
hpp_path = "include/operator/grid_operator.hpp"
cpp_path = "src/operator/grid_operator.cpp"
hpp_dir = "include/operator"
cpp_dir = "src/operator"

# 读取原始头文件和实现文件
with open(hpp_path, "r") as f:
    hpp_content = f.read()
with open(cpp_path, "r") as f:
    cpp_content = f.read()

# 匹配所有子类声明
class_pattern = re.compile(
    r'class\s+(\w+Operator)\s*:\s*public\s+GridOperator\s*\{(.*?)\};',
    re.DOTALL
)
classes = class_pattern.findall(hpp_content)

# 匹配基类声明
base_class_pattern = re.compile(
    r'(class\s+GridOperator\s*\{.*?^\};)', re.DOTALL | re.MULTILINE
)
base_class_match = base_class_pattern.search(hpp_content)
base_class_decl = base_class_match.group(1) if base_class_match else ""

# 提取 include 区域
include_pattern = re.compile(r'^(#include.*?)(?=namespace|\nclass)', re.DOTALL | re.MULTILINE)
include_match = include_pattern.search(hpp_content)
include_block = include_match.group(1) if include_match else ""

# 生成每个子类的 .hpp 和 .cpp 文件
for class_name, class_body in classes:
    # 提取构造函数参数
    ctor_pattern = re.compile(rf'{class_name}\s*\((.*?)\);', re.DOTALL)
    ctor_match = ctor_pattern.search(class_body)
    ctor_args = ctor_match.group(1).replace('\n', ' ').strip() if ctor_match else ""

    # 提取成员变量
    member_pattern = re.compile(r'(?:private:|protected:)(.*)', re.DOTALL)
    member_match = member_pattern.search(class_body)
    member_vars = member_match.group(1).strip() if member_match else ""

    # 提取方法声明
    method_pattern = re.compile(r'(public:.*?)(?:private:|protected:)', re.DOTALL)
    method_match = method_pattern.search(class_body)
    method_decls = method_match.group(1).replace('public:', '').strip() if method_match else ""

    # 生成 .hpp 文件内容
    hpp_code = f"""#pragma once
{include_block}

namespace VXZ {{

class {class_name} : public GridOperator {{
public:
    explicit {class_name}({ctor_args});
    {method_decls}
private:
    {member_vars}
}};

}}
"""
    # 写入 .hpp 文件
    with open(os.path.join(hpp_dir, f"{class_name}.hpp"), "w") as f:
        f.write(hpp_code)

    # 匹配实现
    impl_pattern = re.compile(
        rf'// {class_name} implementation(.*?)(?=// \w+Operator implementation|// End of namespace|}} // namespace VXZ|\Z)',
        re.DOTALL
    )
    impl_match = impl_pattern.search(cpp_content)
    impl_code = impl_match.group(1).strip() if impl_match else "// TODO: 未找到实现"

    # 生成 .cpp 文件内容
    cpp_code = f"""#include "operator/{class_name}.hpp"

namespace VXZ {{

{impl_code}

}}
"""
    # 写入 .cpp 文件
    with open(os.path.join(cpp_dir, f"{class_name}.cpp"), "w") as f:
        f.write(cpp_code)

# 生成新的 grid_operator.hpp
new_hpp = f"""#pragma once

{include_block}

"""
for class_name, _ in classes:
    new_hpp += f'#include "operator/{class_name}.hpp"\n'

new_hpp += f"""

namespace VXZ {{

{base_class_decl}

}}
"""

with open(hpp_path, "w") as f:
    f.write(new_hpp)

print("所有子类 .hpp/.cpp 文件和 grid_operator.hpp 已自动补全并生成。")
