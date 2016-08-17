#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_test");
  if (argc != 7){
    ROS_INFO("usage: currently, adds the norm of two vectors x y z x y z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_ray_trace::RayTrace>("/gazebo_simulation/ray_trace");





  gazebo_ray_trace::RayTrace srv;
  srv.request.start.x = atof(argv[1]);
  srv.request.start.y = atof(argv[2]);
  srv.request.start.z = atof(argv[3]);

  srv.request.end.x = atof(argv[4]);
  srv.request.end.y = atof(argv[5]);
  srv.request.end.z = atof(argv[6]);



  if(client.call(srv)){
    ROS_INFO("Distance  %f", srv.response.dist);
  }else{
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
  












