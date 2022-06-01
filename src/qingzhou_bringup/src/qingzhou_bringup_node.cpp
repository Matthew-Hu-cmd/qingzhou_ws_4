

#include "qingzhou_bringup/qingzhou_bringup.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "qingzhou_bringup");
  ros::NodeHandle nh;
  actuator node(nh);
  node.run();
  return 0;
}

