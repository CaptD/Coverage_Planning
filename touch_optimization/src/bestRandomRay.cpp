/**
 *  Plots a ray and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"

#include "gazebo_ray_trace/calcEntropy.h"
#include "gazebo_ray_trace/plotRayUtils.h"
#include "gazebo_ray_trace/TouchOptimizeGrid.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include <random>



void randomSelection(PlotRayUtils &plt)
{
  tf::Point best_start, best_end;

  double bestIG;
  bestIG = 0;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> rand(0.0,1.0);


  for(int i=0; i<500; i++){
    tf::Point start(rand(gen), rand(gen), rand(gen));
    tf::Point end(rand(gen)+.5, rand(gen)+.5, rand(gen)+.5);
    end.normalized();
    double IG = plt.getIG(start, end, 0.01, 0.002);
    if (IG > bestIG){
      bestIG = IG;
      best_start = start;
      best_end = end;
    }
    
  }


  plt.plotCylinder(best_start, best_end, 0.01, 0.002);
  ROS_INFO("Ray is: %f, %f, %f.  %f, %f, %f", 
	   best_start.getX(), best_start.getY(), best_start.getZ(),
	   best_end.getX(), best_end.getY(), best_end.getZ());
  
}


void bestRayUsingMessages()
{
  tf::Point best_start, best_end;
  double bestIG;
  bestIG = 0;

  PlotRayUtils plt;
  for(int i = -20; i <= 20; i++){
    for(int j = -20; j <= 20; j++){
      double x = 1 + 0.02 * (i);
      double y = 2 + 0.02 * (j);
      tf::Point start(x, y, 3.3);
      tf::Point end(x, y, 2.0);

      // plt.plotCylinder(start, end, 0.01, 0.002);
      double IG = plt.getIG(start, end, 0.01, 0.002);
      if (IG > bestIG){
	bestIG = IG;
	best_start = start;
	best_end = end;
      }

      // ros::Duration(0.1).sleep();
    } 
  }

  plt.plotCylinder(best_start, best_end, 0.01, 0.002);
}

void bestRayUsingSingleService()
{
  PlotRayUtils plt;
  ros::NodeHandle n_;
  ros::ServiceClient client_optimize_ = 
    n_.serviceClient<gazebo_ray_trace::TouchOptimizeGrid>
    ("/gazebo_simulation/optimize_grid");

  gazebo_ray_trace::TouchOptimizeGrid srv;
  
  
  srv.request.direction.x = 0;
  srv.request.direction.y = 0;
  srv.request.direction.z = -1;
  srv.request.x_start = 0.6;
  srv.request.x_end   = 1.4;
  srv.request.x_bins  = 41;

  srv.request.y_start = 1.6;
  srv.request.y_end   = 2.4;
  srv.request.y_bins  = 41;

  srv.request.z_start = 3.3;
  srv.request.z_end   = 3.3;
  srv.request.z_bins  = 1;

  srv.request.err_radius = 0.01;
  srv.request.err_dist = 0.002;

  tf::transformStampedTFToMsg( plt.getTrans(), srv.request.trans);

  if(!client_optimize_.call(srv)){
    ROS_ERROR("Ray Trace Failed");
  }

  ROS_INFO("response %f", srv.response.best.x);

  tf::Point best_start;
  tf::Point best_end;
  tf::pointMsgToTF(srv.response.best, best_start);
  tf::vector3MsgToTF(srv.request.direction, best_end);
  best_end = best_end + best_start;
  
  plt.plotCylinder(best_start, best_end, 0.01, 0.002);

  // plt.plotCylinder(srv.response.best, srv.response.best + srv.request.direction, 0.01, 0.002);

}



int main(int argc, char **argv){

  ros::init(argc, argv, "best_random_ray");
  PlotRayUtils plt;
  // bestRayUsingMessages();
  // bestRayUsingSingleService();
  for(int i=0; i < 5; i++){
    randomSelection(plt);
  }

  return 0;
}
