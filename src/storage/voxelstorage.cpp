#include "storage/voxelstorage.hpp"
#include <stdexcept>
#include <fstream>
#include <unordered_map>
#include <queue>
#include <cstring>

namespace VXZ {

// VoxelStorageFactory implementation
// std::unique_ptr<VoxelStorage> VoxelStorageFactory::create(
//     const std::string& type,
//     const VXZ::VoxelGrid& grid
// ) {
//     // if (type == "svo") {
//     //     return std::make_unique<SVOStorage>(grid);        
//     // }
     
//     //else if (type == "ssvdag") {
//     //     return std::make_unique<SSVDAGStorage>(grid);
//     // } else if (type == "svdag") {
//     //     return std::make_unique<SVDAGStorage>(grid);
//     // } else if (type == "openvdb") {
//     //     return std::make_unique<OpenVDBStorage>(grid);
//     // } else if (type == "nanovdb") {
//     //     return std::make_unique<NanoVDBStorage>(grid);
//     // } else {
//     //     throw std::invalid_argument("Unknown storage type: " + type);
//     // }
// }



// ... (rest of NanoVDBStorage implementation)

} // namespace VXZ 