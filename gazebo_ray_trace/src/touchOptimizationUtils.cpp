#include "touchOptimizationUtils.h"



bool TouchOptimizationUtils::bestFromGrid(gazebo_ray_trace::TouchOptimizeGrid::Request &req,
					  gazebo_ray_trace::TouchOptimizeGrid::Response &resp)
{
  tf::StampedTransform trans;
  tf::transformStampedMsgToTF(req.trans, trans);
  
  double bestIG = -1;
  tf::Point best_start;
  // tf::Point start(req.x_start, req.y_start, req.z_start);
  tf::Vector3 direction(req.direction.x, req.direction.y, req.direction.z);
  
  double x_diff = (req.x_bins > 1 ? (req.x_end - req.x_start)/(req.x_bins-1) : 0);
  double y_diff = (req.y_bins > 1 ? (req.y_end - req.y_start)/(req.y_bins-1) : 0);
  double z_diff = (req.z_bins > 1 ? (req.z_end - req.z_start)/(req.z_bins-1) : 0);
  

  for(int x_ind = 0; x_ind < req.x_bins; x_ind++){
    double x = req.x_start + x_ind * x_diff;
    for(int y_ind = 0; y_ind < req.y_bins; y_ind++){
      double y = req.y_start + y_ind * y_diff;
      for(int z_ind = 0; z_ind < req.z_bins; z_ind++){
	double z = req.z_start + z_ind * z_diff;
	tf::Point start(x,y,z);
	tf::Point end = start + direction;
      
	double IG = ray_tracer_->getIG(trans*start, trans*end, req.err_radius, req.err_dist);
	// ROS_INFO("Ray: %f, %f, %f. IG: %f", start.getX(), start.getY(), start.getZ(), IG);
	if (IG > bestIG){
	  bestIG = IG;
	  best_start = start;
	}

      }
    }
  }

  resp.IG = bestIG;
  tf::pointTFToMsg(best_start, resp.best);
  
  // resp.IG = 1.234;
  // resp.best.x = 1.337;
  // resp.best.y = 1.337;
  // resp.best.z = 1.337;

}

void TouchOptimizationUtils::advertiseServices()
{
  srv_grid_ = rosnode_->advertiseService("optimize_grid", 
					 &TouchOptimizationUtils::bestFromGrid, this);

}
