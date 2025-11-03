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

#define SQRT2 1.41421356237
#define SCALE 10.f

#define LENGTH 91 // 9m field
#define WIDTH 61   // 6m field
#define STEP_SIZE 0.5
#define HARD_RADIUS 1 // odd
#define SOFT_RADIUS 3 // odd
#define CIRC_CUTOFF 3

#define expanGroupSize 5000
#define OFF_LIMIT 3

#define BORDER_2M -2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2
#define BORDER_8_9M BORDER_2M,BORDER_2M,BORDER_2M,BORDER_2M,-2,-2,-2,-2,-2,-2,-2,-2,-2
#define BORDER_20M BORDER_2M,BORDER_2M,BORDER_2M,BORDER_2M,BORDER_2M,BORDER_2M,BORDER_2M,BORDER_2M,BORDER_2M,BORDER_2M

#define FIELD_2M 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
#define FIELD_20M FIELD_2M,FIELD_2M,FIELD_2M,FIELD_2M,FIELD_2M,FIELD_2M,FIELD_2M,FIELD_2M,FIELD_2M,FIELD_2M

#define GOAL_2_2M -3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3

#define BORDER_ALL BORDER_2M, BORDER_20M, BORDER_2M
#define BORDER_GOAL BORDER_2M, BORDER_8_9M, GOAL_2_2M, BORDER_8_9M, BORDER_2M
#define FIELD_BFB BORDER_2M, FIELD_20M, BORDER_2M

#define BA_14LINES BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL,BORDER_ALL
#define BG_6LINES BORDER_GOAL,BORDER_GOAL,BORDER_GOAL,BORDER_GOAL,BORDER_GOAL,BORDER_GOAL

#define FA_6LINES FIELD_BFB,FIELD_BFB,FIELD_BFB,FIELD_BFB,FIELD_BFB,FIELD_BFB
#define FA_36LINES FA_6LINES,FA_6LINES,FA_6LINES,FA_6LINES,FA_6LINES,FA_6LINES
#define FA_216LINES FA_36LINES,FA_36LINES,FA_36LINES,FA_36LINES,FA_36LINES,FA_36LINES

#define FA_300LINES FA_216LINES,FA_36LINES,FA_36LINES,FA_6LINES,FA_6LINES

//  BA = Border All, BG = Border-Goal-Border, FA = Boarder-Field-Boarder
#define BOARD BA_14LINES, BG_6LINES, FA_300LINES, BG_6LINES, BA_14LINES     // 340 x 240




float final_path[2000]; 
std::vector<std::pair<int, int> > final_p;
float board_cost[LENGTH*WIDTH];
int final_path_size = 0;

/**
 * @brief Converts board index to coordinate pair

 * @param index board index (y*WIDTH + x)
 * @return coordinate pair (x, y)
 */
inline std::pair<int, int> indexToCoord(int index) {
    int y = index / WIDTH;
    int x = index % WIDTH;
    return std::make_pair(x, y);
}
// Returns
std::pair<int*, int> get_obstacles(int* opps, int* teammates, int ballX, int ballY, bool ballIsGoal) {
    int maxOpps = 4;
    int maxTeammates = 3;

    int maxObstacles = maxOpps + maxTeammates;
    if (!ballIsGoal) maxObstacles++;

    int* obstacles = new float[2*maxObstacles];
    int index = 0;
    // Opponents
    for (int i = 0; i < maxOpps; ++i) {
        if (opps[2*i] == -1 && opps[2*i+1] == -1) break;

        obstacles[index] = opps[2*i];
        obstacles[index+1] = opps[2*i+1];
        index += 2;
    }
    // Teammates
    for (int i = 0; i < maxTeammates; ++i) {
        if (teammates[2*i] == -1 && opps[2*i+1] == -1) break;

        obstacles[index] = teammates[2*i];
        obstacles[index+1] = teammates[2*i+1];
        index += 2;
    }
    // Ball
    if (!ballIsGoal) {
        obstacles[index] = ballX;
        obstacles[index+1] = ballY;
        index += 2;
    }

    return std::make_pair(obstacles, index);
}

/**
 * @brief Converts float value array_dist to float value meters

 * necessary??
 */
float float_to_meters(float array_dist){
    return array_dist/SCALE;
}

/**
 * @brief Octile distance is always greater than or equal to the Euclidean distance.

 * @param current position and goal position
 * @return float value octile distance between current position and goal position
*/
float octile_distance(float cur_x, float cur_y, float goalX, float goalY){
    float dx = abs(cur_x - goalX);
    float dy = abs(cur_y - goalY);
    return (dx + dy) + (SQRT2 - 2) * fmin(dx, dy);
}



/**
 * @brief checks if it's possible to go directly from current position to goal position

 * @param current position, goal position, obstacles, number of obstacles
 * @return true if it's possible to go directly from current position to goal position, false otherwise
 */
bool can_go_direct(float cur_x, float cur_y, float goalX, float goalY, float obstacles[], float num_obstacles, float max_safe_dist)
{
    float a = 10.0;
    float b = 10.0*(cur_x-goalX)/(goalY-cur_y);
    float c = -a*cur_x - b*cur_y;

    for (int i = 0; i < num_obstacles; i++)
    {
        float x = float_to_meters(obstacles[i]);
        float y = float_to_meters(obstacles[i+1]);

        float dist_to_line = abs(a*x + b*y + c)/sqrt(a*a + b*b);

        // too far to be a problem on the way
        if (dist_to_line > max_safe_dist)
        {
            continue;
        }

        // use dot product to calculate the 
        // angle between current->obstacle and current->goal
        // if both angles are acute, path is obstructed.
        float dot_cur_to_obst = (x-cur_x)*(goalX-cur_x) + (y-cur_y)*(goalY-cur_y);
        float dot_goal_to_obst = (x-goalX)*(cur_x-goalX) + (y-goalY)*(cur_y-goalY);
        
        // if close obstacle is between the current and goal position, path is obstructed
        if (dot_cur_to_obst > 0 and dot_goal_to_obst > 0)
        {
            // run A* to find optimal path
            return false;
        }

    }

    // no obstacles are in the way
    // should continue with direct path to goal
    return true;
}


/**
 * @brief Builds direct path to goal, checked for no obstacles 

 * @param current position, goal position
 */
void build_direct_path(float cur_x, float cur_y, float goalX, float goalY){
    float xx = goalX - cur_x;
    float yy = goalY - cur_y;

    int num_steps = std::max(abs(xx), abs(yy))/STEP_SIZE;

    float dx = xx/num_steps;
    float dy = yy/num_steps;

    final_path[0] = cur_x;
    final_path[1] = cur_y;
    for(int i = 1; i <= num_steps; i++){
        final_path[2*i] = cur_x + dx*i;
        final_path[2*i +1] = cur_y + dy*i;
    }
    final_path[2*num_steps + 2] = goalX;
    final_path[2*num_steps + 3] = goalY;

    return;
}

/**
 * @brief Modifies the final_p vector 
 * 
 * @param endNode: Node* to the final node in A* path
 * @return nothing
 */
void build_best_path(Node* endNode){
    // reset the vector, but conserving the capacity
    final_p.clear();
    // build best path
    Node* currNode = endNode;
    while(currNode != nullptr) {
        std::pair<int, int> coords = indexToCoord(currNode->boardIndex);
        final_p.push_back(coords);

        currNode = currNode->pathParent;
    }
    std::reverse(final_p.begin(), final_p.end());
}


/**
 * @brief Get the board cost object
 * 
 * @param obstacles[]: array of obstacles
 * @param num_obstacles: number of obstacles
 * @return float* board cost
 */
void get_board_cost(int* obstacles, int numObstacles) {
    // Board cost setup: init to 0 for all states
    for (int i = 0; i < LENGTH*WIDTH; i++){
        board_cost[i] = 0;
    }

    // for each obstacle, increment the cost 
    for (int i = 0; i < 2*numObstacles; i+=2){
        int x = obstacles[i];
        int y = obstacles[i+1];

        // hard radius
        int j = 0;
        for (; j < expanGroupSize && expansion_pos_dist[j] < HARD_RADIUS; j++){
            int xx = x + expansion_pos_x[j];
            int yy = y + expansion_pos_y[j];

            if (xx < 0 || xx >= LENGTH || yy < 0 || yy >= WIDTH){
                continue;
            }
            board_cost[xx*WIDTH + yy] = OFF_LIMIT;
        }

        // soft radius
        float between = SOFT_RADIUS - HARD_RADIUS;
        for (; j < expanGroupSize && expansion_pos_dist[j] < SOFT_RADIUS; j++){
            int xx = x + expansion_pos_x[j];
            int yy = y + expansion_pos_y[j];

            if (xx < 0 || xx >= LENGTH || yy < 0 || yy >= WIDTH){
                continue;
            }

            if (board_cost[xx*WIDTH + yy] != OFF_LIMIT){
                board_cost[xx*WIDTH + yy] += (SOFT_RADIUS - expansion_pos_dist[j])/between;
                board_cost[xx*WIDTH + yy] = std::min(board_cost[xx*WIDTH + yy], float(OFF_LIMIT));
            }
        }
    }

    // TODO: reset the cost of the goal post and distance r around the ball
    //board_cost[int(std::round(ball_X)*WIDTH + std::round(ball_Y))] = 0;

    return;
}

/**
 * @brief Processes neighbor node of current node, adds to open set if 
 * 
 * @param currNode: Node* current node in A* search
 * @param neighborNode: Node* neighbor node of currNode
 * @param goalX: x-coord of goal node
 * @param goalY: y-coord of goal node
 * @param openSet: min heap for A*
 * @return float* board cost
 */
inline void checkNeighbor(
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

    float cost = board_cost[neighY*WIDTH + neighX];
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

/**
 * @brief A* algorithm
 * 
 * @param params[]: opponent info is listed in {[ID], X, Y, Z}
 * @param params_size: 
 */
void astar(int startX, int startY, int goalX, int goalY, int* opps, int* teammates, int ballX, int ballY) {
    // segment param
    bool ballIsGoal = (goalX == ballX && goalY == ballY);
    std::pair<float*, int> obstacleInfo = get_obstacles(opps, teammates, ballX, ballY, ballIsGoal);

    get_board_cost(obstacleInfo.first, obstacleInfo.second);

    Node* board[LENGTH*WIDTH];

    // Intialize preallocated Node* board
    for (int y = 0; y < LENGTH; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            board[y*WIDTH + x] = new Node;
            board[y*WIDTH + x]->pathParent = nullptr;
            board[y*WIDTH + x]->f = std::numeric_limits<float>::infinity();
            board[y*WIDTH + x]->g = std::numeric_limits<float>::infinity();
            board[y*WIDTH + x]->h = std::numeric_limits<float>::infinity();
            board[y*WIDTH + x]->boardIndex = y*WIDTH + x;
        }
    }

    // if(can_go_direct(startX, startY, float goalX, float goalY, float *obstacles, int num_obstacles, float max_safe_dist)){
    //     // build direct path
    //     build_direct_path(int cur_x, int cur_y, int goalX, int goalY)
    //     return;
    // }

    // Initialize start node cost in the node board
    board[startY*WIDTH + startX]->g = 0;
    board[startY*WIDTH + startX]->h = octile_distance(
                                        float(startX), float(startY),
                                        float(goalX), float(goalY)
                                        );
    board[startY*WIDTH + startX]->f = board[startY*WIDTH + startX]->f + board_cost[startY*WIDTH + startX];
    
    std::unordered_set<int> processed;
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openSet;
    openSet.push(board[startY*WIDTH + startX]);

    // A_star
    bool reached = false;
    while (!openSet.empty()) {
        Node* currNode = openSet.top();
        std::pair<int, int> coords= indexToCoord(currNode->boardIndex);
        int x = coords.first;
        int y = coords.second;

        // If reached goal node, build best path
        if (x == goalX && y == goalY) {
            build_best_path(currNode);
            std::cout << "Reached goal node!!" << std::endl;
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
            if (x_i < 0 || x_i >= WIDTH || y_i < 0 || y_i >= LENGTH 
                || processed.find(y_i*WIDTH + x_i) != processed.end()) {
                continue;
            }          
            checkNeighbor(currNode, board[y_i*WIDTH + x_i], goalX, goalY, openSet);     
        }
    }

    // Node board cleanup
    for (int y = 0; y < LENGTH; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            delete board[y*WIDTH + x];
        }

    }
    if (!reached) {
        std::cout << "Failed to reach goal node" << std::endl;
    }
    std::ofstream pathFile("path.txt");
    for (int i = 0; i < final_p.size(); i++) {
        pathFile << final_p[i].first << " " << final_p[i].second << std::endl;
    }
    pathFile.close();

    std::ofstream processFile("processed.txt");
    for (auto it = processed.begin(); it != processed.end(); ++it) {
        std::pair<int, int> coords = indexToCoord(*it);
        processFile << coords.first << " " << coords.second << std::endl;
    }
    processFile.close();
}

/*
https://link.springer.com/chapter/10.1007/978-3-031-28469-4_26
https://docs.google.com/document/d/1aJhwK2iJtU-ri_2JOB8iYvxzbPskJ8kbk_4rb3IK3yc/edit?tab=t.0
*/
