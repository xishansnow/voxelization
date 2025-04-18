#pragma once

#include "voxelstorage.hpp"
#include <memory>
#include <array>

namespace VXZ {

/**
 * @brief Node structure for Sparse Voxel Octree (SVO)
 */
struct SVONode {
    bool is_leaf;
    bool value;
    std::array<std::unique_ptr<SVONode>, 8> children;

    SVONode() : is_leaf(false), value(false) {}
};

/**
 * @brief Implementation of Sparse Voxel Octree storage
 */
class SVOStorage : public VoxelStorage {
public:
    SVOStorage();
    ~SVOStorage() override = default;

    bool save(const std::string& filename) const override;
    bool load(const std::string& filename) override;
    size_t get_size() const override;
    bool to_voxel_grid(VXZ::VoxelGrid& grid) const override;

    /**
     * @brief Build SVO from a voxel grid
     * @param grid The voxel grid to convert
     * @return true if successful, false otherwise
     */
    bool from_voxel_grid(const VXZ::VoxelGrid& grid);

private:
    std::unique_ptr<SVONode> root_;
    size_t max_depth_;
    size_t resolution_;

    /**
     * @brief Recursively build SVO node
     * @param node Current node
     * @param grid Reference to voxel grid
     * @param x,y,z Current position in grid
     * @param size Current node size
     * @param depth Current depth
     */
    void build_node(SVONode* node, const VXZ::VoxelGrid& grid,
                   size_t x, size_t y, size_t z,
                   size_t size, size_t depth);

    /**
     * @brief Recursively save node to file
     * @param node Current node
     * @param ofs Output file stream
     */
    void save_node(const SVONode* node, std::ofstream& ofs) const;

    /**
     * @brief Recursively load node from file
     * @param node Current node
     * @param ifs Input file stream
     */
    void load_node(SVONode* node, std::ifstream& ifs);

    /**
     * @brief Recursively convert node to voxel grid
     * @param node Current node
     * @param grid Reference to voxel grid
     * @param x,y,z Current position in grid
     * @param size Current node size
     */
    void node_to_grid(const SVONode* node, VXZ::VoxelGrid& grid,
                     size_t x, size_t y, size_t z,
                     size_t size) const;

    /**
     * @brief Calculate memory usage of a node
     * @param node Current node
     * @return Size in bytes
     */
    size_t calculate_node_size(const SVONode* node) const;
};

} // namespace VXZ 