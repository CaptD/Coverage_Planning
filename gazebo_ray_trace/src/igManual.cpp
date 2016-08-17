
/**
 *  Plots a grid of rays and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"

#include "calcEntropy.h"
#include "plotRayUtils.h"
#include <sstream>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "ig_manual");
  if (argc != 7){
    ROS_INFO("usage: two vectors x y z x y z");
    return 1;
  }

  PlotRayUtils plt;
  ros::Duration(0.5).sleep();

  tf::Point start(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  tf::Point end(atof(argv[4]), atof(argv[5]), atof(argv[6]));


  plt.plotCylinder(start, end, 0.01, 0.002);
  ros::Duration(0.5).sleep();
  return 0;
}
  












