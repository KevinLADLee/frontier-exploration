#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "frontier_exploration.h"

int main( int argc, char** argv )
{
  ros::init(argc, argv, "frontier_exploration_node");
  frontier::FrontierExploration frontier_exploration;
  ros::spin();
  return(0);
}