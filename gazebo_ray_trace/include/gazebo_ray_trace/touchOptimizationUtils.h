#ifndef __TOUCH_OPTIMIZATION_UTILS__
#define __TOUCH_OPTIMIZATION_UTILS__

#include "ros/ros.h"
#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <boost/thread.hpp>
#include "rayTracePluginUtils.h"
#include "gazebo_ray_trace/TouchOptimizeGrid.h"

class TouchOptimizationUtils{
 private:
  ros::ServiceServer srv_grid_;
  

  bool bestFromGrid(gazebo_ray_trace::TouchOptimizeGrid::Request &req,
		    gazebo_ray_trace::TouchOptimizeGrid::Response &resp);

 public:
  RayTracePluginUtils* ray_tracer_;
  ros::NodeHandle* rosnode_;
  
  void advertiseServices();

};




#endif
