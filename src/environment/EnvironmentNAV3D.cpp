
// EnvironmentNAV3D.cpp
#include "environment/EnvironmentNAV3D.hpp"
#include <cmath>
#include <sbpl/utils/key.h>
#include <openvdb/openvdb.h>
#include <openvdb/io/Stream.h>

EnvironmentNAV3D::EnvironmentNAV3D(double x_min, double x_max,
                                   double y_min, double y_max,
                                   double z_min, double z_max,
                                   double resolution,
                                   HeuristicType h_type)
    : minx_(x_min), maxx_(x_max),
      miny_(y_min), maxy_(y_max),
      minz_(z_min), maxz_(z_max),
      resolution_(resolution), h_type_(h_type),
      startID_(-1), goalID_(-1)
{
    size_x_ = static_cast<int>(std::ceil((maxx_ - minx_) / resolution_));
    size_y_ = static_cast<int>(std::ceil((maxy_ - miny_) / resolution_));
    size_z_ = static_cast<int>(std::ceil((maxz_ - minz_) / resolution_));
    grid_.assign(size_x_ * size_y_ * size_z_, 0);
    InitMotionPrimitives(true);
    setNumDims(3);
    setGridSize(size_x_, size_y_, size_z_);
}

bool EnvironmentNAV3D::InitializeEnv()
{
    return size_x_ > 0 && size_y_ > 0 && size_z_ > 0;
}

void EnvironmentNAV3D::SetStart(double wx, double wy, double wz)
{
    int ix = (wx - minx_) / resolution_, iy = (wy - miny_) / resolution_, iz = (wz - minz_) / resolution_;
    startID_ = StateIDFromCoord(ix, iy, iz);
    setStart(startID_);
}
void EnvironmentNAV3D::SetGoal(double wx, double wy, double wz)
{
    int ix = (wx - minx_) / resolution_, iy = (wy - miny_) / resolution_, iz = (wz - minz_) / resolution_;
    goalID_ = StateIDFromCoord(ix, iy, iz);
    setGoal(goalID_);
}
void EnvironmentNAV3D::SetObstacle(int ix, int iy, int iz)
{
    if (ix >= 0 && iy >= 0 && iz >= 0 && ix < size_x_ && iy < size_y_ && iz < size_z_)
        grid_[ToIndex(ix, iy, iz)] = 1;
}

bool EnvironmentNAV3D::LoadFromOctoMap(const std::string &filename)
{
    octomap::OcTree tree(filename);
    for (auto it = tree.begin(tree.getTreeDepth()); it != tree.end(); ++it)
        if (tree.isNodeOccupied(*it))
        {
            auto c = it.getCoordinate();
            int ix = (c.x() - minx_) / resolution_, iy = (c.y() - miny_) / resolution_, iz = (c.z() - minz_) / resolution_;
            if (ix >= 0 && iy >= 0 && iz >= 0 && ix < size_x_ && iy < size_y_ && iz < size_z_)
                grid_[ToIndex(ix, iy, iz)] = 1;
        }
    return true;
}
bool EnvironmentNAV3D::SaveToOctoMap(const std::string &filename) const
{
    octomap::OcTree tree(resolution_);
    for (int iz = 0; iz < size_z_; ++iz)
        for (int iy = 0; iy < size_y_; ++iy)
            for (int ix = 0; ix < size_x_; ++ix)
                if (grid_[ToIndex(ix, iy, iz)])
                    tree.updateNode(octomap::point3d(minx_ + ix * resolution_, miny_ + iy * resolution_, minz_ + iz * resolution_), true);
    tree.writeBinary(filename);
    return true;
}

bool EnvironmentNAV3D::LoadFromOpenVDB(const std::string &filename, const std::string &gridName)
{
    openvdb::initialize();
    openvdb::io::File file(filename);
    file.open();
    auto base = file.readGrid(gridName);
    auto bgrid = openvdb::gridPtrCast<openvdb::BoolGrid>(base);
    for (auto iter = bgrid->cbeginValueOn(); iter.test(); ++iter)
    {
        auto c = iter.getCoord();
        if (c.x() >= 0 && c.y() >= 0 && c.z() >= 0 && c.x() < size_x_ && c.y() < size_y_ && c.z() < size_z_)
            grid_[ToIndex(c.x(), c.y(), c.z())] = 1;
    }
    file.close();
    return true;
}
bool EnvironmentNAV3D::SaveToOpenVDB(const std::string &filename, const std::string &gridName) const
{
    openvdb::initialize();
    auto bgrid = openvdb::BoolGrid::create(false);
    bgrid->setName(gridName);
    bgrid->setTransform(openvdb::math::Transform::createLinearTransform(resolution_));
    for (int iz = 0; iz < size_z_; ++iz)
        for (int iy = 0; iy < size_y_; ++iy)
            for (int ix = 0; ix < size_x_; ++ix)
                if (grid_[ToIndex(ix, iy, iz)])
                    bgrid->tree().setValue(openvdb::Coord(ix, iy, iz), true);
    openvdb::io::File file(filename);
    file.write({bgrid});
    file.close();
    return true;
}

int EnvironmentNAV3D::GetGoalHeuristic(int s)
{
    int ix, iy, iz;
    GridCoordFromStateID(s, ix, iy, iz);
    int gx, gy, gz;
    GridCoordFromStateID(goalID_, gx, gy, gz);
    return ComputeHeuristic(ix, iy, iz, gx, gy, gz);
}
int EnvironmentNAV3D::GetStartHeuristic(int s)
{
    int ix, iy, iz;
    GridCoordFromStateID(s, ix, iy, iz);
    int sx, sy, sz;
    GridCoordFromStateID(startID_, sx, sy, sz);
    return ComputeHeuristic(ix, iy, iz, sx, sy, sz);
}
int EnvironmentNAV3D::GetFromToHeuristic(int s1, int s2)
{
    int ix1, iy1, iz1, ix2, iy2, iz2;
    GridCoordFromStateID(s1, ix1, iy1, iz1);
    GridCoordFromStateID(s2, ix2, iy2, iz2);
    return ComputeHeuristic(ix1, iy1, iz1, ix2, iy2, iz2);
}
void EnvironmentNAV3D::GetSuccs(int s, std::vector<int> *succ, std::vector<int> *cost)
{
    succ->clear();
    cost->clear();
    int ix, iy, iz;
    GridCoordFromStateID(s, ix, iy, iz);
    for (auto &m : motions_)
    {
        int dx, dy, dz, c;
        std::tie(dx, dy, dz, c) = m;
        int nx = ix + dx, ny = iy + dy, nz = iz + dz;
        if (nx < 0 || ny < 0 || nz < 0 || nx >= size_x_ || ny >= size_y_ || nz >= size_z_)
            continue;
        if (grid_[ToIndex(nx, ny, nz)])
            continue;
        int ns = StateIDFromCoord(nx, ny, nz);
        succ->push_back(ns);
        cost->push_back(c);
    }
}
void EnvironmentNAV3D::GetPreds(int t, std::vector<int> *pred, std::vector<int> *cost) { GetSuccs(t, pred, cost); }

int EnvironmentNAV3D::ComputeHeuristic(int ix, int iy, int iz, int gx, int gy, int gz) const
{
    int dx = abs(ix - gx), dy = abs(iy - gy), dz = abs(iz - gz);
    if (h_type_ == MANHATTAN)
        return dx + dy + dz;
    return static_cast<int>(std::sqrt(dx * dx + dy * dy + dz * dz));
}
void EnvironmentNAV3D::InitMotionPrimitives(bool use26)
{
    motions_.clear();
    std::vector<std::tuple<int, int, int>> b6 = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (auto &d : b6)
        motions_.push_back({std::get<0>(d), std::get<1>(d), std::get<2>(d), static_cast<int>(resolution_)});
    if (use26)
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                    if (abs(dx) + abs(dy) + abs(dz) > 1)
                        motions_.push_back({dx, dy, dz, static_cast<int>(std::sqrt(dx * dx + dy * dy + dz * dz) * resolution_)});
}
