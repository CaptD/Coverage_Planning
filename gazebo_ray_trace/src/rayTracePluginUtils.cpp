#include "rayTracePluginUtils.h"




void RayTracePluginUtils::advertiseServices()
{
  // ROS_INFO("ADVERTIZING SERVICES");
  srv_ = rosnode_->advertiseService("ray_trace", &RayTracePluginUtils::rayTrace, this);

  srv_each_ = rosnode_->advertiseService("ray_trace_each_particle", 
					 &RayTracePluginUtils::rayTraceEachParticle, this);

  srv_cylinder_ = rosnode_->
    advertiseService("ray_trace_cylinder", &RayTracePluginUtils::rayTraceCylinder, this);

  srv_condDisEntropy_ = rosnode_->
    advertiseService("ray_trace_condDisEntropy", &RayTracePluginUtils::rayTraceCondDisEntropy, this);

}

void RayTracePluginUtils::setParticles(geometry_msgs::PoseArray p)
{
  particles_ = p;
}

/**
 *  Performs ray tracing on the loaded world, returning the distace to intersection
 */
double RayTracePluginUtils::rayTrace(math::Vector3 start, math::Vector3 end, gazebo::physics::RayShapePtr ray_){
  double dist;
  std::string entityName;

  ray_->SetPoints(start, end);
  ray_->GetIntersection(dist, entityName);

  return dist;
}

double RayTracePluginUtils::rayTrace(tf::Vector3 start, tf::Vector3 end)
{
  gazebo::physics::RayShapePtr ray_;
  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine(); 
  engine->InitForThread();

  ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
    (engine->CreateShape("ray", gazebo::physics::CollisionPtr()));
  return rayTrace(vectorTFToGazebo(start), vectorTFToGazebo(end), ray_);
}

tf::Vector3 RayTracePluginUtils::calcNormal(tf::Vector3 start, tf::Vector3 end){
  tf::Vector3 direction = end-start;
  double distance = 0.0005;
  std::vector<tf::Vector3> orthog = getOrthogonalBasis(direction);
  
  tf::Point p1 = start + direction*rayTrace(start, end);
  start = start+orthog[0]*distance;
  end = end+orthog[0]*distance;
  tf::Point p2 = start + direction*rayTrace(start,end);

  start = start+orthog[1]*distance;
  end = end+orthog[1]*distance;
  tf::Point p3 = start + direction*rayTrace(start,end);
  return (p3-p1).cross(p2-p1).normalize();  
}

/**
 *  Return vector of two Vector3s that form a basis for the space orthogonal to the ray
 */
std::vector<tf::Vector3> RayTracePluginUtils::getOrthogonalBasis(tf::Vector3 ray)
{
  // ROS_INFO("Orthogonal Parts of %f, %f, %f", ray.getX(), ray.getY(), ray.getZ());
  ray.normalize();
  std::vector<tf::Vector3> v;

  //Initialize vector on the most orthogonal axis
  switch(ray.closestAxis()){
  case 0:
    v.push_back(tf::Vector3(0,0,1));
    v.push_back(tf::Vector3(0,1,0));
    break;
  case 1:
    v.push_back(tf::Vector3(0,0,1));
    v.push_back(tf::Vector3(1,0,0));
    break;
  case 2:
  default:
    v.push_back(tf::Vector3(0,1,0));
    v.push_back(tf::Vector3(1,0,0));
    break;
  }

  //Recover the pure orthogonal parts
  for(int i = 0; i < 2; i++){
    v[i] = (v[i] - ray * ray.dot(v[i])).normalize();
    // ROS_INFO("%f, %f, %f", v[i].getX(), v[i].getY(), v[i].getZ());
  }

  return v;
}

/**
 *  Casts many rays in a cylinder to the part and returns a list of the rays 
 *   produced with all the intersections to particles
 */
std::vector<RayTracePluginUtils::RayIntersection> 
RayTracePluginUtils::rayTraceCylinderHelper(tf::Point start, 
					    tf::Point end, 
					    double err)
{
  tf::Vector3 ray = end-start;
  std::vector<tf::Vector3> ray_orthog = getOrthogonalBasis(ray);
  std::vector<RayIntersection> rays;
  int n = 12;
  rays.resize(n);
      
  for(int i = 0; i < n; i ++){
    double theta = 2*3.1415 * i / n;
    tf::Vector3 offset = err * (ray_orthog[0]*sin(theta) + ray_orthog[1]*cos(theta));
	
    tf::pointTFToMsg(start + offset, rays[i].start);
    tf::pointTFToMsg(end + offset, rays[i].end);
	
    rays[i].dist = rayTraceAllParticles(rays[i].start, rays[i].end);
  }
      
  return rays;
}


std::vector<CalcEntropy::ConfigDist> RayTracePluginUtils::intersectionsToConfig(std::vector<RayIntersection> const &rays, double depth_err)
{
 std::vector<CalcEntropy::ConfigDist> distToConfig;
  for(int ray_index = 0; ray_index < rays.size(); ray_index ++){
    for(int id=0; id < rays[ray_index].dist.size(); id++){
      int n = 0; //For adding extra points to simulate error
      for(int i=-1*n; i <=n; i++){
	double err = depth_err * i/n;
	CalcEntropy::ConfigDist c;
	c.id = id;
	c.dist = rays[ray_index].dist[id];
	distToConfig.push_back(c);
      }
    }
  }
  return distToConfig;
}


/**
 *  Service for calculating the conditional discrete entropy of a touch
 */
bool RayTracePluginUtils::rayTraceCondDisEntropy(gazebo_ray_trace::RayTraceCylinder::Request &req,
						 gazebo_ray_trace::RayTraceCylinder::Response &resp)
{
  tf::Vector3 start(req.start.x, req.start.y, req.start.z);
  tf::Vector3 end(req.end.x, req.end.y, req.end.z);

  std::vector<RayIntersection> rays = rayTraceCylinderHelper(start, end, req.error_radius);

  std::vector<CalcEntropy::ConfigDist>  distToConfig = intersectionsToConfig(rays, req.error_depth);

  tf::Vector3 normal = calcNormal(start, end);
  double cosOffNormal = fabs(normal.dot((end-start).normalized()));
  // ROS_INFO("Along Normal: %f", cosOffNormal);

  cosOffNormal = std::max(cosOffNormal, 0.05); //Limit off angle to avoid overflow errors
  double dist_err = req.error_depth/cosOffNormal;

  resp.IG = CalcEntropy::calcIG(distToConfig, dist_err, rays[0].dist.size());
  // ROS_INFO("IG calculated");
  resp.rays.resize(rays.size());
  for(int i = 0; i < rays.size(); i++){
    resp.rays[i].start = rays[i].start;
    resp.rays[i].end = rays[i].end;
    resp.rays[i].dist = rays[i].dist;
  }
}

double RayTracePluginUtils::getIG(tf::Point start, tf::Point end,
				  double err_radius, double err_dist)
{
  std::vector<RayIntersection> rays = rayTraceCylinderHelper(start, end, err_radius);
  // ROS_INFO("There are %d intersections", rays[0].dist.size());    
  ROS_INFO("About to call distToConfig");


  std::vector<CalcEntropy::ConfigDist>  distToConfig= intersectionsToConfig(rays, err_dist);
  ROS_INFO("Called distToConfig");
  return CalcEntropy::calcIG(distToConfig, err_dist, rays[0].dist.size());
}

/**
 *  Service for casting many rays in a cylinder to the world
 */
bool RayTracePluginUtils::rayTraceCylinder(gazebo_ray_trace::RayTraceCylinder::Request &req,
					   gazebo_ray_trace::RayTraceCylinder::Response &resp)
{
  tf::Vector3 start(req.start.x, req.start.y, req.start.z);
  tf::Vector3 end(req.end.x, req.end.y, req.end.z);

  std::vector<RayIntersection> rays = rayTraceCylinderHelper(start, end, req.error_radius);

  resp.rays.resize(rays.size());
  for(int i = 0; i < rays.size(); i++){
    resp.rays[i].start = rays[i].start;
    resp.rays[i].end = rays[i].end;
    resp.rays[i].dist = rays[i].dist;
  }
}

/**
 *  Service for Ray tracing in the loaded world.
 */
bool RayTracePluginUtils::rayTrace(gazebo_ray_trace::RayTrace::Request &req,
				   gazebo_ray_trace::RayTrace::Response &resp)
{
  math::Vector3 start, end;

  start.x = req.start.x;
  start.y = req.start.y;
  start.z = req.start.z;

  end.x = req.end.x;
  end.y = req.end.y;
  end.z = req.end.z;



  gazebo::physics::RayShapePtr ray_;
  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine(); 
  engine->InitForThread();

  ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
    (engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  // resp.dist = rayTrace(start, end, ray_);
  resp.dist = rayTrace(start, end, ray_);
  // ROS_INFO("Traced ray and responded with distance: %f", resp.dist);
  return true;
}

/**
 *  DEPRICATED because differential entropy is no longer used
 *  Service for Ray tracing each particle and 
 *   return the entropy of the distribution
 */
bool RayTracePluginUtils::rayTraceEntropy(gazebo_ray_trace::RayTraceEntropy::Request &req,
					  gazebo_ray_trace::RayTraceEntropy::Response &resp)
{
  std::vector<double> dist = rayTraceAllParticles(req.start, req.end);
  resp.entropy = CalcEntropy::calcDifferentialEntropy(dist);
  return true;
}

/**
 *  Service for Ray traces each particles
 */
bool RayTracePluginUtils::rayTraceEachParticle(gazebo_ray_trace::RayTraceEachParticle::Request &req,
					       gazebo_ray_trace::RayTraceEachParticle::Response &resp)
{
  resp.dist = rayTraceAllParticles(req.start, req.end);
  // ROS_INFO("Traced ray and responded with %d different distances", resp.dist.size());
  return true;
}

/** 
 *  Ray traces each particle.
*   Distances are returned as an array.
*   Order of distances in the array is the same as the order of the particles
*/
std::vector<double> RayTracePluginUtils::rayTraceAllParticles(geometry_msgs::Point startm, 
							      geometry_msgs::Point endm)
{
  tf::Vector3 start, end;

  start.setX(startm.x);
  start.setY(startm.y);
  start.setZ(startm.z);

  end.setX(endm.x);
  end.setY(endm.y);
  end.setZ(endm.z);
  return rayTraceAllParticles(start, end);
}

/**
 *  Ray Traces all particles
 */
std::vector<double> RayTracePluginUtils::rayTraceAllParticles(tf::Point start, 
							      tf::Point end)
{
  tf::Transform trans;
  // ROS_INFO("Setting up ray");
  gazebo::physics::RayShapePtr ray_;
  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine(); 
  engine->InitForThread();

  ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
    (engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

	   
  std::vector<double> dist;
  dist.resize(particles_.poses.size());
  for(int i=0; i<particles_.poses.size(); i++){

    tf::Vector3 v = tf::Vector3(particles_.poses[i].position.x,
				particles_.poses[i].position.y,
				particles_.poses[i].position.z);
    trans.setOrigin(v);
    tf::Quaternion q;
    tf::quaternionMsgToTF(particles_.poses[i].orientation, q);
    trans.setRotation(q);
    trans = trans.inverse();
	

    dist[i] = rayTrace(vectorTFToGazebo(trans*start), 
				   vectorTFToGazebo(trans*end), 
				   ray_);
  }
  // ROS_INFO("Returning Ray");
  return dist;
}

math::Vector3 RayTracePluginUtils::vectorTFToGazebo(const tf::Vector3 t)
{
  math::Vector3 v;
  v.x = t.getX();
  v.y = t.getY();
  v.z = t.getZ();
  return v;
}
