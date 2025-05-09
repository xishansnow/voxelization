// ThetaStarPlanner.h
#ifndef THETASTARPLANNER_H
#define THETASTARPLANNER_H

#include "EnvironmentNAV3D.h"
#include <vector>
#include <tuple>
#include <unordered_map>
#include <queue>

// 3D Theta* Planner
class ThetaStarPlanner {
public:
    ThetaStarPlanner(EnvironmentNAV3D* env);
    ~ThetaStarPlanner() = default;

    // Plan path within given time (s)
    std::vector<int> Plan(double max_time_seconds);

private:
    struct Node {
        int id;
        double g, f;
        int parent;
    };

    struct NodeCmp {
        bool operator()(const Node& a, const Node& b) const { return a.f > b.f; }
    };

    EnvironmentNAV3D* env_;
    std::unordered_map<int, Node> nodes_;  // stateID -> Node

    bool LineOfSight(int id1, int id2) const;
    void GetNeighbors(int stateID, std::vector<std::pair<int,int>>& neighs) const;
    double Heuristic(int id1, int id2) const;
};

#endif // THETASTARPLANNER_H

