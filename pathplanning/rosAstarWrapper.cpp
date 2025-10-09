#include <ros/ros.h> //will need for using ROS
#include <std_msgs> //ROS library for subcriber

#include "Astar.h"


class AstarROSWrapper {
    public:
        AstarROSWrapper : {} //default initializer

        /** 
         * @brief Astar using ROS
         * 
         * @param nh this is the NodeHandle which we can use to subcribe to a particular topic
        
        */
        void callAstar(ros::NodeHandle *nh) {
            //depending on how the ROBOT is handled, we can split this up into multiple topics to subcribe to


            robot_position_subscriber_ = nh->subcribe(
                "robot_position", 2, callbackRobotPosition, this);

            goal_position_subscriber_ = nh->subscribe(
                "goal_position", 2, callbackGoalPosition, this);
            
            //this subcriber will expect two items
            //!std_msgs does not have a float options so we should encode the opponent position in a different manner
            //update the number of expected values to get
            opponent_position_subscriber_ = nh->subscribe(
                "all_opponents_position", 2, callbackOpponentPositions, this); 
            
            //subscribe to more topics to get relevant data to call the Astar algorithm

            //TODO astar should return a path that the robot can then post
            astar(robotx, roboty, goalx, goaly, param, params_size);
            
        }

        //change std_msgs::Int32 depending on the robot coordinate
        void callbackRobotPosition(const std_msgs::Int32MultiArray &msg) { //msg.data should be an array
            //access msg using msg.data
            //this will have access to the robots coordinates
            robotx = msg.data.at(0);
            roboty = msg.data.at(1);
        }

        void callbackGoalPosition(const std_msgs::Int32MultiArray &msg) {
            goalx = msg.data.at(0);
            goaly = msg.data.at(1);
        }

        void callbackOpponentPositions(const std_msgs::Int32MultiArray &msg) {
            //update the param and params_size 

        }

        void resetValues() {
            robotx = 0;
            roboty = 0;
            goalx  = 0;
            goaly  = 0;

            //param = 
            //params_size = 
        }
        
    private:
        int robotx = 0;
        int roboty = 0;

        int goalx = 0;
        int goaly = 0;

        float* param;
        int params_size;

        
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "Astar_algo"); //create a node for the Astar algorithm
    ros::NodeHandle nh;


    ros::spin(); //keeps the node alive

    //call anything that needs to be cleaned up here after the node stops
}