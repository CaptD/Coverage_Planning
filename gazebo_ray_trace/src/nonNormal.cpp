/**
 *  Plots a ray and the intersections of that ray with obstacles 
 *   I mostly use this for quick testing of plots
 */

#include "ros/ros.h"
#include <tf/tf.h>
#include "calcEntropy.h"
#include "plotRayUtils.h"
#include <sstream>
#include <math.h>


int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_test");
  PlotRayUtils plt;
  ROS_INFO("Hello, this is a testing program");

  // tf::Vector3 v(0,0,-1);

  // ROS_INFO("FurthestAxis: %d", v.closestAxis());


  //Start and end vectors of the ray
  // tf::Point start(1.5, 2, 3.5);
  // tf::Point end(1.5, 2, 2.5);

  // plt.plotCylinder(start, end, 0.05, 0.002);

  // plt.plotRay(start, end);


  //---------------
  // RADIAL SPOKES
  //---------------
  // //This was a quick script for casting rays in a circle
  // for(int i = 0; i <= 3; i++){
  //   double radius = .15 * i;
  //   for(int j = 0; j < 8; j++){
  //     double theta = (2*3.1415 * j)/8;
  //     double x = 1 + radius * cos(theta);
  //     double y = 2 + radius * sin(theta);
  //     plt.plotCylinder(tf::Point(x,y, 3.5),
  // 		       tf::Point(x,y, 2.5),
  // 		   0.01, 0.001);
      

  //     ros::Duration(.1).sleep();
  //   }
  // }


  //--------------
  // NON NORMAL
  //-----------------
  // plt.plotCylinder(tf::Point(1.5, 2, 3.5),
  // 		     tf::Point(1.5, 2, 2.5),
  // 		   0.01, 0.001);
  // plt.plotCylinder(tf::Point(1.5, 3, 3.5),
  // 		     tf::Point(1.5, 1, 2.5),
  // 		   0.01, 0.001);


  //-------------
  // OFF EDGE
  //-------------
  plt.plotCylinder(tf::Point(1.57, 1.7, 3.5),
		   tf::Point(1.57, 1.7, 2.5),
  		   0.01, 0.001);
  plt.plotCylinder(tf::Point(1.58, 1.9, 3.5),
		   tf::Point(1.58, 1.9, 2.5),
  		   0.01, 0.001);
  plt.plotCylinder(tf::Point(1.59, 2.1, 3.5),
		   tf::Point(1.59, 2.1, 2.5),
  		   0.01, 0.001);
  plt.plotCylinder(tf::Point(1.6, 2.3, 3.5),
		   tf::Point(1.6, 2.3, 2.5),
  		   0.01, 0.001);

  return 0;
}
