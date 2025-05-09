// LazyThetaStarPlanner.h
#ifndef LAZY_THETASTAR_PLANNER_H
#define LAZY_THETASTAR_PLANNER_H

#include "EnvironmentNAV3D.h"
#include <vector>
#include <unordered_map>
#include <queue>

typedef int StateID;

// Lazy Theta* Planner for 3D environments
class LazyThetaStarPlanner {
public:
    LazyThetaStarPlanner(EnvironmentNAV3D* env);
    ~LazyThetaStarPlanner() = default;

    // Plan path within given time limit (seconds)
    std::vector<StateID> Plan(double max_time_seconds);

private:
    struct Node {
        StateID id;
        double g, f;
        StateID parent;
        bool closed;
    };

    struct NodeCmp {
        bool operator()(const Node* a, const Node* b) const {
            return a->f > b->f;
        }
    };

    EnvironmentNAV3D* env_;  
    std::unordered_map<StateID, Node> nodes_;  
    std::priority_queue<Node*, std::vector<Node*>, NodeCmp> open_;  

    void Initialize(StateID startID, StateID goalID);
    void ImprovePath(StateID goalID, double max_time_seconds);
    bool LineOfSight(StateID s1, StateID s2) const;
    double Heuristic(StateID s1, StateID s2) const;
    void GetSuccessors(StateID s, std::vector<std::pair<StateID,int>>& succs) const;
};

#endif // LAZY_THETASTAR_PLANNER_H
