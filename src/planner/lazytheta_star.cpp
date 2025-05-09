// LazyThetaStarPlanner.cpp
#include "LazyThetaStarPlanner.h"
#include <chrono>
#include <cmath>
#include <iostream>

LazyThetaStarPlanner::LazyThetaStarPlanner(EnvironmentNAV3D* env) : env_(env) {}

void LazyThetaStarPlanner::Initialize(StateID startID, StateID goalID) {
    nodes_.clear();
    while(!open_.empty()) open_.pop();
    Node startNode{startID, 0.0, Heuristic(startID, goalID), startID, false};
    nodes_[startID] = startNode;
    open_.push(&nodes_[startID]);
}

void LazyThetaStarPlanner::ImprovePath(StateID goalID, double max_time_seconds) {
    using Clock = std::chrono::steady_clock;
    auto start_time = Clock::now();
    while(!open_.empty()) {
        if(std::chrono::duration<double>(Clock::now() - start_time).count() > max_time_seconds)
            break;
        Node* s = open_.top(); open_.pop();
        if(s->closed) continue;
        if(s->id == goalID) break;
        s->closed = true;

        std::vector<std::pair<StateID,int>> succs;
        GetSuccessors(s->id, succs);
        for(auto& [succID, cost] : succs) {
            Node* succNode;
            if(nodes_.count(succID)==0) {
                Node newNode{succID, INFINITY, INFINITY, -1, false};
                nodes_[succID] = newNode;
            }
            succNode = &nodes_[succID];
            if(!succNode->closed) {
                StateID parent = s->parent;
                double tentative_g;
                if(LineOfSight(parent, succID)) {
                    double dist = Heuristic(parent, succID);
                    tentative_g = nodes_[parent].g + dist;
                    if(tentative_g < succNode->g) {
                        succNode->parent = parent;
                        succNode->g = tentative_g;
                    }
                } else {
                    tentative_g = s->g + cost;
                    if(tentative_g < succNode->g) {
                        succNode->parent = s->id;
                        succNode->g = tentative_g;
                    }
                }
                succNode->f = succNode->g + Heuristic(succID, goalID);
                open_.push(succNode);
            }
        }
    }
}

std::vector<StateID> LazyThetaStarPlanner::Plan(double max_time_seconds) {
    StateID startID = env_->getStartStateID();
    StateID goalID  = env_->getGoalStateID();
    Initialize(startID, goalID);
    ImprovePath(goalID, max_time_seconds);

    std::vector<StateID> path;
    if(nodes_.count(goalID) && nodes_[goalID].parent!=-1) {
        StateID cur = goalID;
        while(cur != startID) {
            path.push_back(cur);
            cur = nodes_[cur].parent;
        }
        path.push_back(startID);
        std::reverse(path.begin(), path.end());
    }
    return path;
}

bool LazyThetaStarPlanner::LineOfSight(StateID s1, StateID s2) const {
    int x0,y0,z0, x1,y1,z1;
    env_->GridCoordFromStateID(s1, x0,y0,z0);
    env_->GridCoordFromStateID(s2, x1,y1,z1);
    // Bresenham as in ThetaStar
    int dx = abs(x1-x0), dy = abs(y1-y0), dz = abs(z1-z0);
    int sx = x1>x0?1:-1, sy = y1>y0?1:-1, sz = z1>z0?1:-1;
    int err1, err2, dx2=dx*2, dy2=dy*2, dz2=dz*2;
    int cx=x0, cy=y0, cz=z0;
    auto check=[&](){ return env_->IsObstacle(env_->StateIDFromCoord(cx,cy,cz)); };
    if(dx>=dy && dx>=dz) {
        err1=dy2-dx; err2=dz2-dx;
        for(int i=0;i<dx;i++){
            if(err1>0){ cy+=sy; err1-=dx2; }
            if(err2>0){ cz+=sz; err2-=dx2; }
            cx+=sx; err1+=dy2; err2+=dz2;
            if(check()) return false;
        }
    } else if(dy>=dx && dy>=dz) {
        err1=dx2-dy; err2=dz2-dy;
        for(int i=0;i<dy;i++){
            if(err1>0){ cx+=sx; err1-=dy2; }
            if(err2>0){ cz+=sz; err2-=dy2; }
            cy+=sy; err1+=dx2; err2+=dz2;
            if(check()) return false;
        }
    } else {
        err1=dx2-dz; err2=dy2-dz;
        for(int i=0;i<dz;i++){
            if(err1>0){ cx+=sx; err1-=dz2; }
            if(err2>0){ cy+=sy; err2-=dz2; }
            cz+=sz; err1+=dx2; err2+=dy2;
            if(check()) return false;
        }
    }
    return true;
}

double LazyThetaStarPlanner::Heuristic(StateID s1, StateID s2) const {
    return static_cast<double>(env_->GetFromToHeuristic(s1, s2));
}

void LazyThetaStarPlanner::GetSuccessors(StateID s, std::vector<std::pair<StateID,int>>& succs) const {
    std::vector<int> ids; std::vector<int> costs;
    env_->GetSuccs(s, &ids, &costs);
    for(size_t i=0;i<ids.size();++i) succs.emplace_back(ids[i], costs[i]);
}
