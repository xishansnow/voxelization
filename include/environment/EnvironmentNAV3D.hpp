// EnvironmentNAV3D.h
#ifndef ENVIRONMENT_NAV3D_H
#define ENVIRONMENT_NAV3D_H

#include <sbpl/discrete_space_information/environment.h>
#include <vector>
#include <string>
#include <octomap/OcTree.h>
#include <openvdb/openvdb.h>

// 3D grid environment class for SBPL with serialization support
class EnvironmentNAV3D : public DiscreteSpaceInformation {
public:
    enum HeuristicType { MANHATTAN, EUCLIDEAN };

    EnvironmentNAV3D(double x_min, double x_max,
                     double y_min, double y_max,
                     double z_min, double z_max,
                     double resolution,
                     HeuristicType h_type = EUCLIDEAN);
    ~EnvironmentNAV3D() override {}

    bool InitializeEnv();
    void SetStart(double wx, double wy, double wz);
    void SetGoal(double wx, double wy, double wz);
    void SetObstacle(int ix, int iy, int iz);

    bool LoadFromOctoMap(const std::string& filename);
    bool SaveToOctoMap(const std::string& filename) const;
    bool LoadFromOpenVDB(const std::string& filename, const std::string& gridName = "Occupancy");
    bool SaveToOpenVDB(const std::string& filename, const std::string& gridName = "Occupancy") const;

    int GetFromToHeuristic(int stateID_from, int stateID_to) override;
    int GetGoalHeuristic(int stateID) override;
    int GetStartHeuristic(int stateID) override;
    void GetSuccs(int sourceStateID, std::vector<int>* succIDs, std::vector<int>* costs) override;
    void GetPreds(int targetStateID, std::vector<int>* predIDs, std::vector<int>* costs) override;

private:
    double minx_, maxx_, miny_, maxy_, minz_, maxz_, resolution_;
    int size_x_, size_y_, size_z_;
    HeuristicType h_type_;
    std::vector<char> grid_;
    int startID_, goalID_;
    std::vector<std::tuple<int,int,int,int>> motions_;

    inline int ToIndex(int ix, int iy, int iz) const;
    inline void FromIndex(int idx, int& ix, int& iy, int& iz) const;
    int ComputeHeuristic(int ix, int iy, int iz, int gx, int gy, int gz) const;
    void InitMotionPrimitives(bool use_26_neighbors);
};

#endif // ENVIRONMENT_NAV3D_H


