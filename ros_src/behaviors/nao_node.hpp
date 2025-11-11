#ifndef NAO_BEHAVIOR_NODE
#define NAO_BEHAVIOR_NODE

#include "rclcpp/rclcpp.hpp"
// #include "wbk_interfaces/vec_position.hpp"
#include "../../behaviors/naobehavior.h"

using namespace std;

class NaoBehaviorNode : public rclcpp::Node, public NaoBehavior {
public:
  NaoBehaviorNode(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);
  // ~Ros2NaoBehavior() = 0;
};
#endif