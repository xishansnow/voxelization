
// ThetaStarPlanner.cpp
#include "planner/theta_star.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

ThetaStarPlanner::ThetaStarPlanner(EnvironmentNAV3D* env) : env_(env) {}

std::vector<int> ThetaStarPlanner::Plan(double max_time_seconds) {
    using Clock = std::chrono::steady_clock;
    auto start_time = Clock::now();

    int startID = env_->getStartStateID();
    int goalID  = env_->getGoalStateID();

    auto& startNode = nodes_[startID];
    startNode.id = startID;
    startNode.g = 0.0;
    startNode.f = Heuristic(startID, goalID);
    startNode.parent = startID;

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;
    open.push(startNode);
    std::unordered_map<int,bool> closed;

    while(!open.empty()) {
        if(std::chrono::duration<double>(Clock::now() - start_time).count() > max_time_seconds) break;
        Node curr = open.top(); open.pop();
        if(closed[curr.id]) continue;
        closed[curr.id] = true;
        if(curr.id == goalID) {
            // reconstruct path
            std::vector<int> path;
            int id = goalID;
            while(id != startID) {
                path.push_back(id);
                id = nodes_[id].parent;
            }
            path.push_back(startID);
            std::reverse(path.begin(), path.end());
            return path;
        }
        std::vector<std::pair<int,int>> neighbors;
        GetNeighbors(curr.id, neighbors);
        for(auto& nb : neighbors) {
            int succID = nb.first;
            int cost   = nb.second;
            double tentative_g;
            int bestParent;
            if(LineOfSight(curr.parent, succID)) {
                bestParent = curr.parent;
                double dist = env_->GetFromToHeuristic(curr.parent, succID);
                tentative_g = nodes_[curr.parent].g + dist;
            } else {
                bestParent = curr.id;
                tentative_g = curr.g + cost;
            }
            auto it = nodes_.find(succID);
            if(it == nodes_.end() || tentative_g < it->second.g) {
                Node nbNode;
                nbNode.id = succID;
                nbNode.g = tentative_g;
                nbNode.f = tentative_g + Heuristic(succID, goalID);
                nbNode.parent = bestParent;
                nodes_[succID] = nbNode;
                open.push(nbNode);
            }
        }
    }
    // failure
    return {};
}

bool ThetaStarPlanner::LineOfSight(int id1, int id2) const {
    int x0,y0,z0, x1,y1,z1;
    env_->GridCoordFromStateID(id1, x0,y0,z0);
    env_->GridCoordFromStateID(id2, x1,y1,z1);
    int dx = abs(x1 - x0), dy = abs(y1 - y0), dz = abs(z1 - z0);
    int sx = (x1 > x0) ? 1 : -1;
    int sy = (y1 > y0) ? 1 : -1;
    int sz = (z1 > z0) ? 1 : -1;
    int err1, err2;
    int dx2 = dx*2, dy2 = dy*2, dz2 = dz*2;
    int cx = x0, cy = y0, cz = z0;
    if(dx >= dy && dx >= dz) {
        err1 = dy2 - dx; err2 = dz2 - dx;
        for(int i=0; i<dx; ++i) {
            if(err1 > 0) { cy += sy; err1 -= dx2; }
            if(err2 > 0) { cz += sz; err2 -= dx2; }
            cx += sx;
            err1 += dy2; err2 += dz2;
            if(env_->IsObstacle(env_->StateIDFromCoord(cx,cy,cz))) return false;
        }
    } else if(dy >= dx && dy >= dz) {
        err1 = dx2 - dy; err2 = dz2 - dy;
        for(int i=0; i<dy; ++i) {
            if(err1 > 0) { cx += sx; err1 -= dy2; }
            if(err2 > 0) { cz += sz; err2 -= dy2; }
            cy += sy;
            err1 += dx2; err2 += dz2;
            if(env_->IsObstacle(env_->StateIDFromCoord(cx,cy,cz))) return false;
        }
    } else {
        err1 = dx2 - dz; err2 = dy2 - dz;
        for(int i=0; i<dz; ++i) {
            if(err1 > 0) { cx += sx; err1 -= dz2; }
            if(err2 > 0) { cy += sy; err2 -= dz2; }
            cz += sz;
            err1 += dx2; err2 += dy2;
            if(env_->IsObstacle(env_->StateIDFromCoord(cx,cy,cz))) return false;
        }
    }
    return true;
}

void ThetaStarPlanner::GetNeighbors(int stateID, std::vector<std::pair<int,int>>& neighs) const {
    std::vector<int> succIDs;
    std::vector<int> costs;
    env_->GetSuccs(stateID, &succIDs, &costs);
    for(size_t i=0;i<succIDs.size();++i) neighs.emplace_back(succIDs[i], costs[i]);
}

double ThetaStarPlanner::Heuristic(int id1, int id2) const {
    return static_cast<double>(env_->GetFromToHeuristic(id1, id2));
}
