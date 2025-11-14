#define ASTAR_H

#define FIELD_LENGTH 9 // meters
#define FIELD_WIDTH 6  // meters
#define GRID_RESOLUTION 10 // grid spaces per meter
#define BORDER_SIZE 2 // meters
#define AV_FIELD_X 30
#define AV_FIELD_Y 20
#define AV_COST_BOARD_X AV_FIELD_X * GRID_RESOLUTION
#define AV_COST_BOARD_Y AV_FIELD_Y * GRID_RESOLUTION

#define TOTAL_LENGTH (FIELD_LENGTH + 2 * BORDER_SIZE)
#define TOTAL_WIDTH (FIELD_WIDTH + 2 * BORDER_SIZE)
#define COST_BOARD_LENGTH (int)(TOTAL_LENGTH * GRID_RESOLUTION)
#define COST_BOARD_WIDTH (int)(TOTAL_WIDTH * GRID_RESOLUTION)

#define MAX_PATH_LENGTH 300
#define MAX_OBSTACLES 22

#define HARD_RADIUS 0.3f
#define SOFT_RADIUS 1.0f
#define CIRC_CUTOFF 3
#define expanGroupSize 5000
#define OFF_LIMIT_COST 1000.0f

#define SQRT2 1.41421356237 
#define SCALE 10.0f
#define STEP_SIZE 0.5
#define CIRC_CUTOFF 3
#include <vector>
#include <utility>
#include <queue>
#include <iostream>
#include <limits>

#include "../math/vecposition.h"
#include "../worldmodel/worldmodel.h"

struct Node {
    Node* pathParent;
   
    float f; // g + h
    float g;
    float h;
    int boardIndex;

    Node() : pathParent(nullptr),
             f(std::numeric_limits<float>::infinity()), 
             g(std::numeric_limits<float>::infinity()),
             h(std::numeric_limits<float>::infinity()), 
             boardIndex(0) {}

    Node(int index, float g_cost, float h_cost) 
        : pathParent(nullptr), f(g_cost + h_cost), 
          g(g_cost), h(h_cost), boardIndex(index) {}
};

struct CompareNodes {
    bool operator()(Node* const& a, Node* const& b) const {
        // Min heap - larger values have lower priority
        // Tie breaker, if f costs are the same, compare h costs
        if (a->f == b->f) {
            return a->h > b->h;
        }
        return a->f > b->f;  
    }
};


class PathPlanning {
    public:
        PathPlanning();

        ~PathPlanning();

        VecPosition* findPathAV(const VecPosition& start, 
                         const VecPosition& goal,
                         WorldModel* worldModel);
        
        // std::vector<std::pair<int, int>> findPath(int startX, int startY, 
        //               int goalX, int goalY, 
        //               int* opps, int* teammates,
        //               int ballX, int ballY);
            

    private:
        float costBoard[AV_COST_BOARD_X*AV_COST_BOARD_Y];
        // Original implmentation path vector
        //std::vector<std::pair<int, int>> finalPath;
        // Final path for integrating with ATVilla codebase
        VecPosition finalPathAV[MAX_PATH_LENGTH];
        VecPosition obstacles[MAX_OBSTACLES];

        int pathLength = 0;
        int numObstacles = 0;
        
        void buildBestPathAV(Node* node);

        // void createCostBoard(float obstacles[], int num_obstacles);

        void getObstaclesAV(WorldModel* worldModel);
        void createCostBoardAV();
        float octile_distance(float curX, float curY, float goalX, float goalY);
        inline void checkNeighborAV(Node* currNode, Node* neighborNode, 
                                    int goalX, int goalY, 
                                    std::priority_queue<Node*, std::vector<Node*>, 
                                    CompareNodes>& openSet);
        inline std::pair<int, int> vecPositionToCoord(VecPosition vec);
        inline VecPosition coordToVecPosition(int x, int y);
        inline std::pair<int, int> indexToCoord(int index);

};



// extern void build_direct_path(float cur_x, float cur_y, float goal_x, float goal_y);
// extern float* get_obstacles(float* opponent_input, int num_inputs);

// extern void astar(int start_x, int start_y, int goal_x, int goal_y, float* param, int params_size);

