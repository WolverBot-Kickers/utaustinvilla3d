#include <errno.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <fstream>
#include <rcssnet/tcpsocket.hpp>
//#include <rcssnet/udpsocket.hpp>
#include <rcssnet/exception.hpp>
#include <netinet/in.h>
#include "behaviors/behavior.h"
#include "behaviors/naobehavior.h"
#include "optimization/optimizationbehaviors.h"
#include "behaviors/pkbehaviors.h"
#include "behaviors/simplesoccer.h"
#include "behaviors/gazebobehavior.h"
#include "stats/recordstatsbehavior.h"
#include "rclcpp/rclcpp.h"

// class NaoBehaviorWrapper : rclcpp::Node
// {

// }
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared(rclcpp::Node));
  rclcpp::shutdown();
  return 0;
}
