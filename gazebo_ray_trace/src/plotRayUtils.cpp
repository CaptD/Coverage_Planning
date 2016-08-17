#include "plotRayUtils.h"

#include "gazebo_ray_trace/RayTrace.h"
#include "gazebo_ray_trace/RayTraceEachParticle.h"
#include "gazebo_ray_trace/RayTraceEntropy.h"
#include "gazebo_ray_trace/RayTraceCylinder.h"
#include "gazebo_ray_trace/TouchOptimizeGrid.h"
#include "calcEntropy.h"

PlotRayUtils::PlotRayUtils()
{
  marker_pub_ = 
    n_.advertise<visualization_msgs::Marker>("ray_trace_markers", 1000);
  marker_pub_array_ = 
    n_.advertise<visualization_msgs::MarkerArray>("ray_trace_markers_array", 10);
  client_ray_trace_ = 
    n_.serviceClient<gazebo_ray_trace::RayTrace>
    ("/gazebo_simulation/ray_trace");
  client_ray_trace_particles_ = 
    n_.serviceClient<gazebo_ray_trace::RayTraceEachParticle>
    ("/gazebo_simulation/ray_trace_each_particle");
  client_ray_trace_cylinder_ = 
    n_.serviceClient<gazebo_ray_trace::RayTraceCylinder>
    ("/gazebo_simulation/ray_trace_cylinder");
  client_ray_trace_condDisEntropy_ = 
    n_.serviceClient<gazebo_ray_trace::RayTraceCylinder>
    ("/gazebo_simulation/ray_trace_condDisEntropy");


  tf_listener_.waitForTransform("/my_frame", "/particle_frame", ros::Time(0), ros::Duration(10.0));
  tf_listener_.lookupTransform("/particle_frame", "/my_frame", ros::Time(0), trans_);

  intersect_index_ = 0;
  ray_index_ = 0;
}

tf::StampedTransform PlotRayUtils::getTrans()
{
  //TODO: Update when new transform becomes available
  return trans_;
}

/** 
 * Plots a point as a red dot
 */
void PlotRayUtils::plotIntersection(tf::Point intersection, int index){
  marker_pub_.publish(getIntersectionMarker(intersection, index));
}

/**
 *  Plots intersections of a ray with all particles as red dots
 */
void PlotRayUtils::plotIntersections(std::vector<double> dist, 
				     tf::Point rayStart, tf::Point rayEnd,
				     bool overwrite)
{
  visualization_msgs::MarkerArray m;
  if(!overwrite){
    intersect_index_ += dist.size();
  }

  int point_id = intersect_index_;
  for(int i = 0; i < dist.size(); i++){
    tf::Point intersection = rayStart + dist[i] * (rayEnd-rayStart).normalized();
    m.markers.push_back(getIntersectionMarker(intersection, point_id));
    point_id++;
  }
  marker_pub_array_.publish(m);
}

void PlotRayUtils::plotIntersections(tf::Point rayStart, tf::Point rayEnd, bool overwrite)
{
  plotIntersections(getDistToParticles(rayStart, rayEnd), rayStart, rayEnd, overwrite);
}

visualization_msgs::Marker PlotRayUtils::getIntersectionMarker(tf::Point intersection, int index){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray_intersection";
  marker.id = index;
 
  marker.type = visualization_msgs::Marker::SPHERE;
 
  marker.action = visualization_msgs::Marker::ADD;
 
  tf::pointTFToMsg(intersection, marker.pose.position);

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
 
  marker.lifetime = ros::Duration();
  return marker;
}

/*
 * publishes a visualization message with the ray.
 *  By default overwrites the previous array
 */
void PlotRayUtils::plotRay(tf::Point start, tf::Point end, bool overwrite)
{
  if(!overwrite){
    ray_index_++;
  }
  visualization_msgs::Marker marker = createRayMarker(start, end, ray_index_);

  //wait until subscribed
  ros::Rate poll_rate(100);
  int i = 0;
  while(marker_pub_.getNumSubscribers() == 0 && i < 100){
    poll_rate.sleep();
    i++;
  }
  marker_pub_.publish(marker);
}


void PlotRayUtils::labelRay(tf::Point start, std::string text){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray_label";
  marker.id = ray_index_;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
 
  tf::pointTFToMsg(start, marker.pose.position);
 
  marker.scale.z = 0.05;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.7f;
  marker.color.a = 1.0;

  marker.text = text;
 
  marker.lifetime = ros::Duration();

  marker_pub_.publish(marker);
}

/**
 * Creates and returns the ray marker
 */
visualization_msgs::Marker PlotRayUtils::createRayMarker(tf::Point start, tf::Point end, 
							 int index)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray";
  marker.id = index;

  marker.type = visualization_msgs::Marker::ARROW;
 
  marker.action = visualization_msgs::Marker::ADD;
 
  
  marker.points.resize(2);
  tf::pointTFToMsg(start, marker.points[0]);
  tf::pointTFToMsg(end, marker.points[1]);
 
  marker.scale.x = 0.005;
  marker.scale.y = 0.04; //Head Width
  // marker.scale.z = 1.0;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.6;
 
  marker.lifetime = ros::Duration();

  return marker;
}

/**  DEPRICATED
 *  Plots ray, intersections, and entropy (text)
 *   DEPRICATED SINCE DIFFERENTIAL ENTROPY IS NO LONGER USED
 */
void PlotRayUtils::plotEntropyRay(tf::Point start, tf::Point end, bool overwrite)
{
  plotRay(start, end, overwrite);
  std::vector<double> dist = getDistToParticles(start, end);
  plotIntersections(dist, start, end, overwrite);
  std::stringstream s;

  s << CalcEntropy::calcDifferentialEntropy(dist);

  labelRay(start, s.str());
}

/**
 * Plots a cylinder of rays and labels the ray with the information gain
 *  radial_err determines the radius of the cylinder
 *  dist_err determines the bin size for calculating the entropy for calculating Information Gain
 */
void PlotRayUtils::plotCylinder(tf::Point start, tf::Point end, double radial_err, double dist_err, bool overwrite)
{

  if(overwrite){
    ray_index_ = intersect_index_ = 0;
  }
    
  gazebo_ray_trace::RayTraceCylinder srv = getIGFullResponse(start, end, radial_err, dist_err);

  tf::Point start_tmp;
  tf::Point end_tmp;

  //Plot all rays used, transforming to world coordinates
  for(int i=0; i<srv.response.rays.size(); i++){
    transformRayToBaseFrame(srv.response.rays[i].start,
			    srv.response.rays[i].end,
			    start_tmp, end_tmp);
		     
    plotRay(start_tmp, end_tmp, overwrite);
    plotIntersections(srv.response.rays[i].dist, start_tmp, end_tmp, overwrite);
    // ros::Duration(0.02).sleep();
  }
  //Plot and label center ray
  plotRay(start, end, false);
  std::stringstream s;
  //Note: IG should always be positive, but I am using fabs here to see errors if IG is negative
  s << (fabs(srv.response.IG) < .0001 ? 0 : srv.response.IG); 
  labelRay(start, s.str());
}

/** 
 *  Calls the rayTracePlugin service to get information gain of a simulated touch
 *   Returns the full response from the service
 */
gazebo_ray_trace::RayTraceCylinder PlotRayUtils::getIGFullResponse(
		  tf::Point start, tf::Point end, double radial_err, double dist_err)
{
  gazebo_ray_trace::RayTraceCylinder srv;

  transformRayToParticleFrame(start, end, srv.request.start, srv.request.end);
  srv.request.error_radius = radial_err;
  srv.request.error_depth = dist_err;

  if(!client_ray_trace_condDisEntropy_.call(srv)){
    ROS_ERROR("Ray Trace Failed");
  }
  return srv;
}

/**
 *  Calls the rayTracePlugin service to get information gain of a simulated touch
 *   Returns the information gain
 */
double PlotRayUtils::getIG(tf::Point start, tf::Point end, double radial_err, double dist_err)
{
  return getIGFullResponse(start, end, radial_err, dist_err).response.IG;
}

/**
 * Call the ros service providedby ray_trace_plugging
 * This servce accepts a ray and returns the transform to the true part
 * particles are not considered
 */
double PlotRayUtils::getDistToPart(tf::Point start, tf::Point end)
{
  gazebo_ray_trace::RayTrace srv;
  transformRayToParticleFrame(start, end, srv.request.start, srv.request.end);

  if(!client_ray_trace_.call(srv)){
    ROS_ERROR("Ray Trace Failed");
  }

  return srv.response.dist;
}

/**
 * Returns the point of intersection with the part along the ray
 *   rays are given in the world frame, 
 *   intersection point is returned in the world frame
 */
bool PlotRayUtils::getIntersectionWithPart(tf::Point start, tf::Point end, tf::Point &intersection)
{
  double dist = getDistToPart(start, end);
  intersection= start + (end-start).normalize() * dist;
  return dist < 999;
}

/**
 * Calls the ros service provided by ray_trace_pluggin.
 *  This service accepts a ray and returns a list of points for where the ray 
 *  intersected each obstacle
 */
std::vector<double> PlotRayUtils::getDistToParticles(tf::Point start, tf::Point end)
{
  gazebo_ray_trace::RayTraceEachParticle srv;
  
  transformRayToParticleFrame(start, end, srv.request.start, srv.request.end);
  
  if(!client_ray_trace_particles_.call(srv)){
    ROS_ERROR("Ray Trace Failed");
  }

  return srv.response.dist;
}

/**
 *  Transform the ray into the particle frame to pass correct ray to gazebo for ray casting
 */
void PlotRayUtils::transformRayToParticleFrame(tf::Point start, tf::Point end, 
					       geometry_msgs::Point &startTransformed,
					       geometry_msgs::Point &endTransformed)
{
  tf::StampedTransform trans = getTrans();
  tf::pointTFToMsg(trans * start, startTransformed);
  tf::pointTFToMsg(trans * end, endTransformed);
}

/**
 *  Transform the ray message into the base frame.
 */
void PlotRayUtils::transformRayToBaseFrame(geometry_msgs::Point start,
					   geometry_msgs::Point end,
					   tf::Point &startBase,
					   tf::Point &endBase)
{
  tf::StampedTransform trans = getTrans();

  tf::pointMsgToTF(start, startBase);
  tf::pointMsgToTF(end,   endBase);
  startBase = trans.inverse() * startBase;
  endBase   = trans.inverse() * endBase;
}
