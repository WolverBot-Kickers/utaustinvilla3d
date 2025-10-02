#define LENGTH 340 // 2m border - 30m field - 2m border
#define WIDTH 240   // 2m border - 20m field - 2m border
#include <queue>
#include <iostream>

struct Node{
    Node* pathParent;
   
    float f; // g + h
    float g;
    float h;
    int boardIndex;
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


extern float final_path[2000];
extern float board_cost[LENGTH*WIDTH];
extern std::vector<std::pair<int, int> > final_p;

extern void build_direct_path(float cur_x, float cur_y, float goal_x, float goal_y);
extern float* get_obstacles(float* opponent_input, int num_inputs);
extern void get_board_cost(float obstacles[], int num_obstacles);
extern void build_best_path(Node* node);
extern void astar(int start_x, int start_y, int goal_x, int goal_y, float* param, int params_size);

