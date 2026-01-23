#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <vector>
#include <fstream>
#include <unordered_set>
#include "Astar.h"
#include "expansion_groups.h"
#include "../math/vecposition.h"


PathPlanning::PathPlanning() {
    // Initialize pathLength and numObstacles
    pathLength = 0;
    numObstacles = 0;
    
    // Initialize costBoard to all zeros
    for (int i = 0; i < AV_COST_BOARD_X * AV_COST_BOARD_Y; i++) {
        costBoard[i] = 0;
    }
}

PathPlanning::~PathPlanning() {
    // Destructor - no dynamic memory to clean up since we're using arrays
    // Could add cleanup code here if needed in the future
}

/**
 * @brief Converts board index to coordinate pair

 * @param index board index (y*WIDTH + x)
 * @return coordinate pair (x, y)
 */
inline std::pair<int, int> PathPlanning::indexToCoord(int index) {
    int y = index / AV_COST_BOARD_X;
    int x = index % AV_COST_BOARD_X;
    return std::make_pair(x, y);
}




/**
 * @brief Octile distance is always greater than or equal to the Euclidean distance.

 * @param current position and goal position
 * @return float value octile distance between current position and goal position
*/
float PathPlanning::octile_distance(float curX, float curY, float goalX, float goalY){
    float dx = abs(curX - goalX);
    float dy = abs(curY - goalY);
    return (dx + dy) + (SQRT2 - 2) * fmin(dx, dy);
}



inline void PathPlanning::checkNeighborAV(
                          Node* currNode, Node* neighborNode, 
                          int goalX, int goalY, 
                          std::priority_queue<Node*, std::vector<Node*>, CompareNodes>& openSet
                        ) {
    // Get coordinates for current node and neighbor node
    std::pair<int, int> currCoords = indexToCoord(currNode->boardIndex);
    std::pair<int, int> neighCoords = indexToCoord(neighborNode->boardIndex);
    int currX = currCoords.first;
    int currY = currCoords.second;
    int neighX = neighCoords.first;
    int neighY = neighCoords.second;

    float cost = costBoard[neighY*AV_COST_BOARD_X + neighX];
    float tempG = currNode->g + octile_distance(float(currX), 
                                                 float(currY),
                                                 float(neighX), 
                                                 float(neighY)
                                                );
    // If new g cost is less than current at neighbor node, update g cost and add to open set
    if (tempG < neighborNode->g) {
        neighborNode->g = tempG;
        neighborNode->h = octile_distance(
                    float(neighX), float(neighY),
                    float(goalX), float(goalY)
                );
        neighborNode->f = neighborNode->g + neighborNode->h + cost; 
        
        neighborNode->pathParent = currNode;
        openSet.push(neighborNode);                                                
    }    
}

std::vector<VecPosition> PathPlanning::findPathAV(const VecPosition& start, const VecPosition& goal, WorldModel* worldModel) {
    getObstaclesAV(worldModel);
    createCostBoardAV();

    Node* board[AV_COST_BOARD_X*AV_COST_BOARD_Y];

    // Intialize preallocated Node* board
    for (int y = 0; y < AV_COST_BOARD_Y; ++y) {
        for (int x = 0; x < AV_COST_BOARD_X; ++x) {
            board[y*AV_COST_BOARD_X + x] = new Node;
            board[y*AV_COST_BOARD_X + x]->boardIndex = y*AV_COST_BOARD_X + x;
        }
    }

    std::pair<int, int> startCoords = vecPositionToCoord(start);
    std::pair<int, int> goalCoords = vecPositionToCoord(goal);
    int startX = startCoords.first;
    int startY = startCoords.second;
    int goalX = goalCoords.first;
    int goalY = goalCoords.second;

    // Initialize start node cost in the node board
    board[startY*AV_COST_BOARD_X + startX]->g = 0;
    board[startY*AV_COST_BOARD_X + startX]->h = octile_distance(float(startX), float(startY), float(goalX), float(goalY));
    board[startY*AV_COST_BOARD_X + startX]->f = board[startY*AV_COST_BOARD_X + startX]->f + costBoard[startY*AV_COST_BOARD_X + startX];
    
    std::unordered_set<int> processed;
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openSet;
    openSet.push(board[startY*AV_COST_BOARD_X + startX]);

    // A_star
    bool reached = false;
    while (!openSet.empty()) {
        Node* currNode = openSet.top();
        std::pair<int, int> coords = indexToCoord(currNode->boardIndex);
        int x = coords.first;
        int y = coords.second;

        // If reached goal node, build best path
        if (x == goalX && y == goalY) {
            buildBestPathAV(currNode);
            // std::cout << "Reached goal node!!" << std::endl;
            reached = true;
            break;
        }
        // Pop current min node
        openSet.pop();
        // If more optimal node at current board position has already been visited and processed, skip
        if (processed.find(currNode->boardIndex) != processed.end()) {
            continue;
        }
        processed.insert(currNode->boardIndex);
        

        const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1}; 
        const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        // Iterate through neighbors
        for (int i = 0; i < 8; ++i) {
            int x_i = x + dx[i];
            int y_i = y + dy[i];
            
            // Check for out of bounds or already processed
            if (x_i < 0 || x_i >= AV_COST_BOARD_X || y_i < 0 || y_i >= AV_COST_BOARD_Y 
                || processed.find(y_i*AV_COST_BOARD_X + x_i) != processed.end()) {
                continue;
            }          
            checkNeighborAV(currNode, board[y_i*AV_COST_BOARD_X + x_i], goalX, goalY, openSet);     
        }
    }
    // Node board cleanup
    for (int y = 0; y < AV_COST_BOARD_Y; ++y) {
        for (int x = 0; x < AV_COST_BOARD_X; ++x) {
            delete board[y*AV_COST_BOARD_X + x];
        }
    }
    // if (!reached) {
    //     std::cout << "Failed to reach goal node" << std::endl;
    // }

    //*return a vector
    std::vector<VecPosition> vec;
    for(int i = 0; i < pathLength; ++i) {
        vec.push_back(finalPathAV[i]);
    }
    return vec;

}

void PathPlanning::getObstaclesAV(WorldModel* worldModel) {
    numObstacles = 0;
    int myNum = worldModel->getUNum();
    // Add all valid opponents to obstacles
    for (int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
        WorldObject* opponent = worldModel->getWorldObject(i);
        if (opponent->validPosition) {
            obstacles[numObstacles] = opponent->pos;
            obstacles[numObstacles].setZ(0);
            ++numObstacles;
        }
    }
    // Add all valid teammates to obstacles, not including self
    for (int i = WO_TEAMMATE1; i <= WO_TEAMMATE11; ++i) {
        if (myNum == i - WO_TEAMMATE1 + 1) {
            continue;
        }
        WorldObject* teammate = worldModel->getWorldObject(i);
        if (teammate->validPosition) {
            obstacles[numObstacles] = teammate->pos;
            obstacles[numObstacles].setZ(0);
            ++numObstacles;
        }
    }
    return;
}

void PathPlanning::createCostBoardAV() {
    // Board cost setup: init to 0 for all states
    for (int i = 0; i <  AV_COST_BOARD_X*AV_COST_BOARD_Y; i++){
        costBoard[i] = 0;
    }

    // for each obstacle, increment the cost 
    for (int i = 0; i < numObstacles; ++i){
        std::pair<int, int> coords = vecPositionToCoord(obstacles[i]);
        int x = coords.first;
        int y = coords.second;

        // hard radius
        int j = 0;
        for (; j < expanGroupSize && expansion_pos_dist[j] < HARD_RADIUS; j++){
            int xx = x + expansion_pos_x[j];
            int yy = y + expansion_pos_y[j];

            if (xx < 0 || xx >= AV_COST_BOARD_X || yy < 0 || yy >= AV_COST_BOARD_Y){
                continue;
            }
            costBoard[yy*AV_COST_BOARD_X + xx] = OFF_LIMIT_COST;
        }

        // soft radius
        float between = SOFT_RADIUS - HARD_RADIUS;
        for (; j < expanGroupSize && expansion_pos_dist[j] < SOFT_RADIUS; j++){
            int xx = x + expansion_pos_x[j];
            int yy = y + expansion_pos_y[j];

            if (xx < 0 || xx >= AV_COST_BOARD_X || yy < 0 || yy >= AV_COST_BOARD_Y){
                continue;
            }

            if (costBoard[yy*AV_COST_BOARD_X + xx] != OFF_LIMIT_COST){
                costBoard[yy*AV_COST_BOARD_X + xx] += (SOFT_RADIUS - expansion_pos_dist[j])/between;
                costBoard[yy*AV_COST_BOARD_X + xx] = std::min(costBoard[yy*AV_COST_BOARD_X + xx], float(OFF_LIMIT_COST));
            }
        }
    }

    return;
}

void PathPlanning::buildBestPathAV(Node* node) {
    pathLength = 0;
    // build best path
    Node* currNode = node;
    while(currNode != nullptr && pathLength < MAX_PATH_LENGTH) {
        std::pair<int, int> coords = indexToCoord(currNode->boardIndex);
        finalPathAV[pathLength] = coordToVecPosition(coords.first, coords.second);

        currNode = currNode->pathParent;
        ++pathLength;
    }
    std::reverse(finalPathAV, finalPathAV + pathLength);
}

inline std::pair<int, int> PathPlanning::vecPositionToCoord(VecPosition vec) {
    return std::make_pair(int(vec.getX()) + int(AV_COST_BOARD_X / 2), int(vec.getY()) + int(AV_COST_BOARD_Y / 2));
}

inline VecPosition PathPlanning::coordToVecPosition(int x, int y) {
    return VecPosition(double(x - int(AV_COST_BOARD_X / 2)), 
                       double(y - int(AV_COST_BOARD_Y / 2)), 
                       0);
}

VecPosition PathPlanning::indexToVecPosition(int index) {
    std::pair<int, int> coords = indexToCoord(index);
    return coordToVecPosition(coords.first, coords.second);
}

int PathPlanning::getPathLength() const {
    return pathLength;
}

/*
https://link.springer.com/chapter/10.1007/978-3-031-28469-4_26
https://docs.google.com/document/d/1aJhwK2iJtU-ri_2JOB8iYvxzbPskJ8kbk_4rb3IK3yc/edit?tab=t.0
*/
