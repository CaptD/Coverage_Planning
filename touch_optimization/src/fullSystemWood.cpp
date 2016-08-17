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
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include "gazebo_ray_trace/plotRayUtils.h"
# define M_PI       3.14159265358979323846  /* pi */

/**
 * Gets initial points for the particle filter by shooting
 * rays at the object
 */
particle_filter::PFilterInit getInitialPoints(PlotRayUtils &plt)
{
  particle_filter::PFilterInit init_points;

  tf::Point start1(1,1,0);
  tf::Point end1(-1,-1,0);
  tf::Point start2(1,1,.2);
  tf::Point end2(-1,-1,.2);
  tf::Point start3(1.1,0.9,0);
  tf::Point end3(-0.9,-1.1,0);
  
  tf::Point intersection;
  plt.getIntersectionWithPart(start1,end1, intersection);
  tf::pointTFToMsg(intersection,  init_points.p1);
  plt.getIntersectionWithPart(start2,end2, intersection);
  tf::pointTFToMsg(intersection, init_points.p2);
  plt.getIntersectionWithPart(start3,end3, intersection);
  tf::pointTFToMsg(intersection, init_points.p3);
  return init_points;
}

void generateRandomTouchWith(tf::Pose &probePose, double tbX, double tbY, double tbZ, double tbRR, double tbRP, double tbRY, double rotateR, double rotateP, double rotateY, double offX, double offY, double offZ)
{

  // tf::Transform rotate;
  // rotate.setRotation(tf::createQuaternionFromRPY(rotateR, rotateP, rotateY));
  tf::Pose touchBase;
  touchBase.setOrigin(tf::Vector3(tbX, tbY, tbZ));
  // std::cout << "RR " << tbRR << ", RP " << tbRP << ", RY " << tbRY << std::cout;
  tf::Quaternion q;
  q.setRPY(tbRR, tbRP, tbRY);

  touchBase.setRotation(q);

  // tf::Pose offset;
  // offset.setOrigin(tf::Vector3(offX, offY, offZ));
  // offset.setRotation(tf::createQuaternionFromRPY(0,0,0));

  // probePose = rotate*offset*touchBase;
  probePose = touchBase;

}

    // [0.6, 0.6, -0.1, 0, 0, 0]
void generateRandomTouchTop(std::mt19937 &gen, tf::Pose &probePose)
{
  std::uniform_real_distribution<double> rand(0,1.0);
  double x_width = 0.2*rand(gen);
  double y_width = 0.7*rand(gen);
  // generateRandomTouchWith(probePose, 
  // 			  .53 + x_width, .4 + y_width, .687, M_PI, 0, 0, 
  // 			  0,0,0,
  // 			  0,0,0);
  generateRandomTouchWith(probePose, 
  			  // 0.8, -0.21, 0.45, M_PI, 0, M_PI, 
  			  0.6 + x_width, -0.41 + y_width, 0.45, M_PI, 0, M_PI, 
  			  0,0,0,
  			  0,0,0);

}

void generateRandomTouchFront(std::mt19937 &gen, tf::Pose &probePose)
{
  std::uniform_real_distribution<double> rand(-1.0,1.0);
  double y_width = 0.05*rand(gen);
  double z_width = 0.05*rand(gen);

  generateRandomTouchWith(probePose, 
			  0.812, -0.050, 0.36, -1.468, -1.396, -2.104,
  			  // 0.812, -0.050, 0.391, -1.396, -2.104, -1.468, 
  			  0,0,0,
  			  0,0,0);
}



void generateRandomTouchFrontRight(std::mt19937 &gen, tf::Pose &probePose)
{
  std::uniform_real_distribution<double> rand(-1.0,1.0);
  double y_width = 0.05*rand(gen);
  double z_width = 0.05*rand(gen);

  generateRandomTouchWith(probePose, 
			  .577, -.611, .360, .398, -1.532, 2.387,
  			  0,0,0,
  			  0,0,0);
}


void generateRandomTouchSide(std::mt19937 &gen, tf::Pose &probePose)
{
  std::uniform_real_distribution<double> rand(-1.0,1.0);
  double x_width = 0.05*rand(gen);
  double z_width = 0.05*rand(gen);
  generateRandomTouchWith(probePose, 
			  .71, .17, .36, 1.58, -1.23, 2.724,
			  // .71, .13, .4,  2.724, -1.23, 1.58,
			  0,0,0,
			  0,0,0);

}



void generateRandomRay(std::mt19937 &gen, tf::Pose &probePose, tf::Point &start, tf::Point &end)
{
  std::uniform_real_distribution<double> rand(0.0, 3.0);
  double faceNum = rand(gen);

  // generateRandomTouchSide(gen, probePose);

  
  if(faceNum < 1.0)
    generateRandomTouchTop(gen, probePose);
  else if(faceNum < 1.5)
    generateRandomTouchFront(gen, probePose);
  else if(faceNum < 2.0)
    generateRandomTouchFrontRight(gen, probePose);
  else
    generateRandomTouchSide(gen, probePose);
  
  tf::Transform probeZ;
  probeZ.setRotation(tf::createQuaternionFromRPY(0,0,0));
  probeZ.setOrigin(tf::Vector3(0,0,0.1));


  // end = (probePose*probeZ).getOrigin();
  // start = tf::Point(0,0,0);
  // end = tf::Transform(probePose.getRotation()) * tf::Point(0,0,.1);


  start = probePose.getOrigin();
  end = probePose.getOrigin() + 
    tf::Transform(probePose.getRotation()) * tf::Point(0,0,.1);
}





/**
 * Randomly chooses vectors, gets the Information Gain for each of 
 *  those vectors, and returns the ray (start and end) with the highest information gain
 */
void randomSelection(PlotRayUtils &plt, tf::Pose &probePose)
{
  // tf::Point best_start, best_end;

  double bestIG;
  bestIG = 0;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> rand(-1.0,1.0);

  tf::Point best_start, best_end;

  for(int i=0; i<500; i++){
    // tf::Point start(rand(gen), rand(gen), rand(gen));
    // start = start.normalize();
    // tf::Point end(rand(gen), rand(gen), rand(gen));
    // end.normalized();
    tf::Point start, end;
    tf::Pose probePoseTmp;
    generateRandomRay(gen, probePoseTmp, start, end);
    // plt.plotRay(start, end);
    double IG = plt.getIG(start, end, 0.01, 0.002);
    if (IG > bestIG){
      
      bestIG = IG;
      best_start = start;
      best_end = end;
      probePose = probePoseTmp;
    }
  }
  plt.plotRay(best_start, best_end);
  plt.plotIntersections(best_start, best_end);
  // plt.plotCylinder(best_start, best_end, 0.01, 0.002, true);
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


geometry_msgs::Pose probeAt(tf::Transform rotate, tf::Transform base, double x, double y, double z, double r, double p, double yaw){
  tf::Pose offset;
  offset.setOrigin(tf::Vector3(x,y,z));
  offset.setRotation(tf::createQuaternionFromRPY(r, p, yaw));
  geometry_msgs::Pose probe_msg;
  tf::poseTFToMsg(rotate*offset*base, probe_msg);
  // probe_pub.publish(probe_msg);
  return probe_msg;

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
  
 

  double dtouch = 0.05;

  tf::Pose probePose;
  geometry_msgs::Pose probe_msg; 

  tf::Point rStart(0.8, -0.21, 0.45);
  tf::Point rEnd(0.8, -0.21, 0.35);


  plt.plotRay(rStart, rEnd);



  for(int i=0; i<7; i++){
    randomSelection(plt, probePose);
    tf::poseTFToMsg(probePose, probe_msg);
    probe_pub.publish(probe_msg);
    ros::topic::waitForMessage<std_msgs::String>(std::string("/process_finished"));
  }


  ROS_INFO("Finished all action");

}
