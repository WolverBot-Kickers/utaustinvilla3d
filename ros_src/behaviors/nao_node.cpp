#include "rclcpp/rclcpp.hpp"
#include "nao_node.hpp"
#include <string>

using namespace std;


NaoBehaviorNode::NaoBehaviorNode(const std::string teamName, int uNum, 
          const map<string, string>& namedParams_, const string& rsg_)
                   : Node(teamName + std::to_string(uNum)), 
                    NaoBehavior( teamName,  uNum, namedParams_,  rsg_)
{

}
