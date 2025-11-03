#ifndef ROS2_NAOBEHAVIOR
#define ROS2_NAOBEHAVIOR

#include "rclcpp/rclcpp.hpp"
// #include "wbk_interfaces/vec_position.hpp"
#include "../behaviors/naobehavior.h"

using namespace std;

class Ros2NaoBehavior : public rclcpp::Node, public NaoBehavior {
public:
  Ros2NaoBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);
  // ~Ros2NaoBehavior() = 0;
};
#endif