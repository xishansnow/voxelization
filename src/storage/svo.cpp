#include "storage/svo.hpp"
#include <fstream>
#include <cmath>

namespace VXZ {

SVOStorage::SVOStorage() : max_depth_(0), resolution_(0) {
    root_ = std::make_unique<SVONode>();
}

bool SVOStorage::save(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) {
        return false;
    }

    // Write header
    ofs.write(reinterpret_cast<const char*>(&max_depth_), sizeof(max_depth_));
    ofs.write(reinterpret_cast<const char*>(&resolution_), sizeof(resolution_));

    // Write tree structure
    save_node(root_.get(), ofs);
    return ofs.good();
}

bool SVOStorage::load(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        return false;
    }

    // Read header
    ifs.read(reinterpret_cast<char*>(&max_depth_), sizeof(max_depth_));
    ifs.read(reinterpret_cast<char*>(&resolution_), sizeof(resolution_));

    // Read tree structure
    load_node(root_.get(), ifs);
    return ifs.good();
}

size_t SVOStorage::get_size() const {
    return calculate_node_size(root_.get());
}

bool SVOStorage::to_voxel_grid(VXZ::VoxelGrid& grid) const {
    if (!root_) {
        return false;
    }
    // Create grid with appropriate resolution
    // grid.resize(resolution_, resolution_, resolution_);
    
    // Convert tree to grid
    node_to_grid(root_.get(), grid, 0, 0, 0, resolution_);
    return true;
}

bool SVOStorage::from_voxel_grid(const VXZ::VoxelGrid& grid) {
    if (grid.get_size_x() != grid.get_size_y() || 
        grid.get_size_x() != grid.get_size_z()) {
        return false; // Only support cubic grids
    }

    resolution_ = grid.get_size_x();
    max_depth_ = static_cast<size_t>(std::log2(resolution_));
    
    // Reset root node
    root_ = std::make_unique<SVONode>();
    
    // Build tree
    build_node(root_.get(), grid, 0, 0, 0, resolution_, 0);
    return true;
}

void SVOStorage::build_node(SVONode* node, const VXZ::VoxelGrid& grid,
                          size_t x, size_t y, size_t z,
                          size_t size, size_t depth) {
    if (size == 1) {
        // Leaf node
        node->is_leaf = true;
        // Use Eigen::Vector3i to get the voxel value
        Eigen::Vector3i position(x, y, z);
        node->value = grid.get(position);
        return;
    }

    size_t half_size = size / 2;
    bool all_same = true;
    bool first_value = grid.get_voxel(x, y, z);

    // Check if all voxels in this region have the same value
    for (size_t i = 0; i < size; ++i) {
        for (size_t j = 0; j < size; ++j) {
            for (size_t k = 0; k < size; ++k) {
                if (grid.get_voxel(x + i, y + j, z + k) != first_value) {
                    all_same = false;
                    break;
                }
            }
            if (!all_same) break;
        }
        if (!all_same) break;
    }

    if (all_same) {
        // All voxels are the same, create leaf node
        node->is_leaf = true;
        node->value = first_value;
        return;
    }

    // Create children nodes
    for (size_t i = 0; i < 8; ++i) {
        size_t child_x = x + ((i & 1) ? half_size : 0);
        size_t child_y = y + ((i & 2) ? half_size : 0);
        size_t child_z = z + ((i & 4) ? half_size : 0);

        node->children[i] = std::make_unique<SVONode>();
        build_node(node->children[i].get(), grid,
                  child_x, child_y, child_z,
                  half_size, depth + 1);
    }
}

void SVOStorage::save_node(const SVONode* node, std::ofstream& ofs) const {
    // Save node type and value
    ofs.write(reinterpret_cast<const char*>(&node->is_leaf), sizeof(node->is_leaf));
    ofs.write(reinterpret_cast<const char*>(&node->value), sizeof(node->value));

    if (!node->is_leaf) {
        // Save children
        for (const auto& child : node->children) {
            if (child) {
                save_node(child.get(), ofs);
            }
        }
    }
}

void SVOStorage::load_node(SVONode* node, std::ifstream& ifs) {
    // Load node type and value
    ifs.read(reinterpret_cast<char*>(&node->is_leaf), sizeof(node->is_leaf));
    ifs.read(reinterpret_cast<char*>(&node->value), sizeof(node->value));

    if (!node->is_leaf) {
        // Load children
        for (auto& child : node->children) {
            child = std::make_unique<SVONode>();
            load_node(child.get(), ifs);
        }
    }
}

void SVOStorage::node_to_grid(const SVONode* node, VXZ::VoxelGrid& grid,
                            size_t x, size_t y, size_t z,
                            size_t size) const {
    if (node->is_leaf) {
        // Fill region with leaf value
        for (size_t i = 0; i < size; ++i) {
            for (size_t j = 0; j < size; ++j) {
                for (size_t k = 0; k < size; ++k) {
                    grid.set(Eigen::Vector3i(x + i, y + j, z + k), node->value);
                }
            }
        }
    } else {
        // Process children
        size_t half_size = size / 2;
        for (size_t i = 0; i < 8; ++i) {
            if (node->children[i]) {
                size_t child_x = x + ((i & 1) ? half_size : 0);
                size_t child_y = y + ((i & 2) ? half_size : 0);
                size_t child_z = z + ((i & 4) ? half_size : 0);

                node_to_grid(node->children[i].get(), grid,
                           child_x, child_y, child_z,
                           half_size);
            }
        }
    }
}

size_t SVOStorage::calculate_node_size(const SVONode* node) const {
    size_t size = sizeof(SVONode);
    
    if (!node->is_leaf) {
        for (const auto& child : node->children) {
            if (child) {
                size += calculate_node_size(child.get());
            }
        }
    }
    
    return size;
}

} // namespace VXZ 