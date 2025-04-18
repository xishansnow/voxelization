# 依赖项安装说明

## Arch Linux / Manjaro 安装指南

在 Arch Linux 中安装依赖项时，建议到 AUR 中查看一下各依赖项的安装目录，并更新 CMakeFileLists.txt 文件中的 CMAKE_PREFIX_PATH 设置

### 系统要求

- Linux 系统（推荐 Arch Linux /Manjaro  或更高版本）
- CMake 2.10 或更高版本
- C++16 兼容的编译器（如 GCC 9+ 或 Clang 10+）
- 
### 一键安装所有依赖

在 Arch Linux 或 Manjaro 系统上，可以使用以下命令一次性安装所有必需的依赖项：

```bash
# 更新系统
sudo pacman -Syu

# 安装基础开发工具
sudo pacman -S base-devel cmake gcc clang

# 安装所有项目依赖
sudo pacman -S eigen openmp openvdb gtest mesa glew glfw-x11
```

### 分步安装

如果您想分步安装或只安装特定的依赖项，可以按照以下步骤进行：

1. 安装基础开发工具：
```bash
sudo pacman -S base-devel
```

2. 安装编译器和构建工具：
```bash
sudo pacman -S gcc clang cmake
```

3. 安装数学库：
```bash
sudo pacman -S eigen
```

4. 安装并行计算支持：
```bash
sudo pacman -S openmp
```

5. 安装体素数据结构支持：
```bash
sudo pacman -S openvdb
```

6. 安装测试框架：
```bash
sudo pacman -S gtest
```

7. 安装图形相关依赖：
```bash
sudo pacman -S mesa glew glfw-x11
```

### 可选依赖

一些可选但推荐安装的包：

```bash
# 安装性能分析工具
sudo pacman -S perf

# 安装调试工具
sudo pacman -S gdb valgrind

# 安装文档工具
sudo pacman -S doxygen graphviz

# 安装代码格式化工具
sudo pacman -S clang-format
```

### 验证安装

安装完成后，可以使用以下命令验证各个组件：

```bash
# 验证 CMake
cmake --version

# 验证 GCC
gcc --version

# 验证 Clang
clang --version

# 验证 pkg-config 是否能找到库
pkg-config --modversion eigen3
pkg-config --modversion openvdb
pkg-config --modversion glfw3
```

## Debian / Ubuntu Linux 安装指南

## 基础依赖

### 1. CMake

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install cmake

# Arch Linux/Manjaro
sudo pacman -S cmake

# 使用 conda
conda install -c conda-forge cmake
```

### 2. Eigen3

```bash
# Ubuntu/Debian
sudo apt-get install libeigen3-dev

# Arch Linux/Manjaro
sudo pacman -S eigen

# 使用 conda
conda install -c conda-forge eigen
```

### 3. OpenMP

```bash
# Ubuntu/Debian
sudo apt-get install libomp-dev

# Arch Linux/Manjaro
sudo pacman -S openmp

# 使用 conda
conda install -c conda-forge openmp
```

### 4. OpenVDB

```bash
# Ubuntu/Debian
sudo apt-get install libopenvdb-dev

# Arch Linux/Manjaro
sudo pacman -S openvdb

# 使用 conda
conda install -c conda-forge openvdb
```

### 5. Google Test

```bash
# Ubuntu/Debian
sudo apt-get install libgtest-dev

# Arch Linux/Manjaro
sudo pacman -S gtest

# 使用 conda
conda install -c conda-forge gtest
```

## 图形相关依赖

### 1. OpenGL

```bash
# Ubuntu/Debian
sudo apt-get install libgl1-mesa-dev

# Arch Linux/Manjaro
sudo pacman -S mesa

# 使用 conda
conda install -c conda-forge mesa-libgl
```

### 2. GLEW

```bash
# Ubuntu/Debian
sudo apt-get install libglew-dev

# Arch Linux/Manjaro
sudo pacman -S glew

# 使用 conda
conda install -c conda-forge glew
```

### 3. GLFW3

```bash
# Ubuntu/Debian
sudo apt-get install libglfw3-dev

# Arch Linux/Manjaro
sudo pacman -S glfw-x11

# 使用 conda
conda install -c conda-forge glfw
```

## 验证安装

安装完成后，可以通过以下命令验证主要依赖项：

```bash
# 验证 CMake
cmake --version

# 验证 Eigen3
pkg-config --modversion eigen3

# 验证 OpenVDB
pkg-config --modversion openvdb

# 验证 OpenGL
glxinfo | grep "OpenGL version"

# 验证 GLFW
pkg-config --modversion glfw3
```

## 常见问题

### 1. CMake 找不到包

如果 CMake 找不到某些包，可以尝试设置环境变量：

```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/path/to/package
```

### 2. 链接错误

如果遇到链接错误，确保所有库都正确安装，并且路径正确：

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/library
```

### 3. 版本冲突

如果遇到版本冲突，可以尝试使用 conda 环境来隔离依赖：

```bash
conda create -n voxelization python=3.8
conda activate voxelization
conda install -c conda-forge cmake eigen openvdb gtest glew glfw
```

### 4. Arch Linux 特定问题

如果在 Arch Linux 上遇到包冲突，可以尝试：

```bash
# 强制刷新包数据库
sudo pacman -Syy

# 升级整个系统
sudo pacman -Syu

# 检查包的依赖关系
pacman -Qi package_name
```

## 参考链接

- [Eigen 官方文档](https://eigen.tuxfamily.org/)
- [OpenVDB 官方文档](https://www.openvdb.org/documentation/)
- [Google Test 文档](https://google.github.io/googletest/)
- [GLFW 文档](https://www.glfw.org/docs/latest/)
- [GLEW 文档](http://glew.sourceforge.net/)
- [Arch Linux 包管理指南](https://wiki.archlinux.org/title/Pacman)
- [Arch Linux Wiki](https://wiki.archlinux.org/) 