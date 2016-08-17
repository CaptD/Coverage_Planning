/**
 *  This is a rosnode that randomly chooses touch points, 
 *    performs simulated measurements, and updates the particle filter.
 *   This node is made to work with a particle filter node, a node that 
 *    publishes visualization messages, and RViz.
 */

#include <ros/ros.h>
#include "particle_filter/PFilterInit.h"
#include "particle_filter/AddObservation.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include "gazebo_ray_trace/plotRayUtils.h"
# define M_PI       3.14159265358979323846  /* pi */



void generateRandomRay(std::mt19937 &gen, tf::Pose &probePose, tf::Point &start, tf::Point &end)
{
  std::uniform_real_distribution<double> rand(-1.0,1.0);

  tf::Pose touchBase;

  touchBase.setOrigin(tf::Vector3(0,0,.05));
  touchBase.setRotation(tf::createQuaternionFromRPY(0, 0, 0));

  tf::Pose offset;
  offset.setOrigin(tf::Vector3(-.25 + rand(gen)*.05, rand(gen)*.05, 0.05));

  offset.setRotation(tf::createQuaternionFromRPY(0,0,0));

  probePose = offset*touchBase;
  tf::Transform probeZ;
  probeZ.setRotation(tf::createQuaternionFromRPY(0,0,0));
  probeZ.setOrigin(tf::Vector3(0,0,-0.1));
  start = probePose.getOrigin();
  end = (probePose * probeZ).getOrigin();
}



/**
 * Randomly chooses vectors, gets the Information Gain for each of 
 *  those vectors, and returns the ray (start and end) with the highest information gain
 */
void randomSelection(PlotRayUtils &plt, tf::Pose &probePose)
{

  double bestIG;
  bestIG = 0;
  std::random_device rd;
  std::mt19937 gen(rd());

  tf::Point best_start, best_end;

  for(int i=0; i<500; i++){
    tf::Point start, end;
    tf::Pose probePoseTmp;
    generateRandomRay(gen, probePoseTmp, start, end);
    double IG = plt.getIG(start, end, 0.01, 0.002);
    // ROS_INFO("Random Ray is: %f, %f, %f.  %f, %f, %f", 
    // 	   start.getX(), start.getY(), start.getZ(),
    // 	   end.getX(), end.getY(), end.getZ());
    
    if (IG > bestIG){
      
      bestIG = IG;
      best_start = start;
      best_end = end;
      probePose = probePoseTmp;
    }
  }

  // plt.plotCylinder(best_start, best_end, 0.01, 0.002, true);
  plt.plotRay(best_start, best_end);
  plt.plotIntersections(best_start, best_end);
  ROS_INFO("Ray is: %f, %f, %f.  %f, %f, %f", 
  	   best_start.getX(), best_start.getY(), best_start.getZ(),
  	   best_end.getX(), best_end.getY(), best_end.getZ());
  
}


bool getIntersection(PlotRayUtils &plt, tf::Point start, tf::Point end, tf::Point &intersection){
  bool intersectionExists = plt.getIntersectionWithPart(start, end, intersection);
  double radius = 0.005;
  intersection = intersection - (end-start).normalize() * radius;
  return intersectionExists;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "updating_particles");
  ros::NodeHandle n;
  PlotRayUtils plt;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> randn(0.0,0.003);

  ROS_INFO("Running...");

  ros::Publisher probe_pub = 
    n.advertise<geometry_msgs::Pose>("/probe_point", 5);
 
  ros::Duration(2).sleep();
  
 


  tf::Pose probePose;
  geometry_msgs::Pose probe_msg; 

  for(int i=0; i<15; i++){
    randomSelection(plt, probePose);
    tf::poseTFToMsg(probePose, probe_msg);
    probe_pub.publish(probe_msg);
    ros::Duration(10.0).sleep();
  }


  ROS_INFO("Finished all action");

}
