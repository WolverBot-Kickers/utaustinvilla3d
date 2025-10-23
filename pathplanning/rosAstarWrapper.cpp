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

            robot_position_subscriber_ = nh->subscribe(
                "robot_position", 2, callbackRobotPosition, this);

            goal_position_subscriber_ = nh->subscribe(
                "goal_position", 2, callbackGoalPosition, this);
            
            //this subcriber will expect two items
            //!std_msgs does not have a float options so we should encode the opponent position in a different manner
            //update the number of expected values to get
            opponent_position_subscriber_ = nh->subscribe(
                "all_opponents_position", 1, callbackOpponentPositions, this); 

            
            teammate_position_subscriber_ = nh->subscribe(
                "all_teammate_position", 1, callbackTeammatePositions, this);

            ball_position_subscriber_ = nh->subscribe(
                "ball_position", 2, callbackBallPosition, this);
            
            
            
            //subscribe to more topics to get relevant data to call the Astar algorithm

            //TODO
            /*
                If we are out of bounds, call builddirectpath to get back onto the field as soon as possible

                else, call astar
            */
            
            //TODO astar should return a path that the robot can then publish
            astar(robotx, roboty, goalx, goaly, opponents, teammates, ballx, bally);


            //should publish the astar path that we want as the topic "astar_path"
            pathPublisher_ = nh->advertise<std_msgs::Int32MultiArray>("astar_path", *path that astar returns*);
            
        }

        //change std_msgs::Int32 depending on the robot coordinate
        void callbackRobotPosition(const std_msgs::Int32MultiArray &msg) { //msg.data should be an array
            //access msg using msg.data
            //this will have access to the robots coordinates
            robotx = msg.data[0];
            roboty = msg.data[1];
        }

        void callbackGoalPosition(const std_msgs::Int32MultiArray &msg) {
            goalx = msg.data[0];
            goaly = msg.data[1];
        }

        //msg: opponent1x opponent1y opponent2x opponent2y ...
        void callbackOpponentPositions(const std_msgs::Int32MultiArray &msg) {
            //update the param and params_size 
            for(size_t i = 0; i < 8; ++i) {
                opponents[i] = msg[i];
            }
        }

        //msg: teammate1x teammate1y teammate2x teammate2y ...
        void callbackTeammatePositions(const std_msgs::Int32MultiArray &msg) {
            //update array for team mate positions
            for(size_t i = 0; i < 6; ++i) {
                teammates[i] = msg[i];
            }
        }

        void callbackBallPosition(const std_msgs::Int32MultiArray &msg) {
            ballx = msg.data[0];
            bally = msg.data[1];
        }

        void resetValues() {
            robotx = 0;
            roboty = 0;
            goalx  = 0;
            goaly  = 0;

            int opponents = {-1,-1,-1,-1,-1,-1,-1,-1};

            int teammates = {-1,-1,-1,-1,-1,-1};
        }
        
    private:
        int robotx = 0;
        int roboty = 0;

        int goalx = 0;
        int goaly = 0;

        int opponents[8] = {-1,-1,-1,-1,-1,-1,-1,-1};

        int teammates[6] = {-1,-1,-1,-1,-1,-1};

        int ballx = 0;
        int bally = 0;

        //should be used for publishing the path we take
        ros::Publisher pathPublisher_;

        
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "Astar_algo"); //create a node for the Astar algorithm
    ros::NodeHandle nh;


    ros::spin(); //keeps the node alive

    ros::shutdown();

    //call anything that needs to be cleaned up here after the node stops
}