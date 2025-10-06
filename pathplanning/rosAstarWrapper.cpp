#include <ros/ros.h> //will need for using ROS
#include <std_msgs> //ROS library

#include "Astar.h"

struct coordinate { //for callbackRobotPosition to return
    float x;
    float y;
}

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

            //subscribe to more topics to get relevant data to call the Astar algorithm
            
        }

        //change std_msgs::Int32 depending on the robot coordinate
        coordinate callbackRobotPosition(const std_msgs::Int32MultiArray &msg) {
            //access msg using msg.data
            //this will have access to the robots coordinates
        }
        
    private:
        
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "Astar_algo"); //create a node for the Astar algorithm
    ros::NodeHandle nh;


    ros::spin(); //keeps the node alive

    //call anything that needs to be cleaned up here after the node stops
}