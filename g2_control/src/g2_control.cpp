#include "g2_control/g2_control.h"
#include <pcl/point_types.h>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

G2Control::G2Control(ros::NodeHandle &nh) {
    _nh_ptr.reset(&nh);

//    nh.param("/g2_control/spin_freq", _spin_freq, 0.0);

//    _g2_pub    = nh.advertise<std_msgs::Float64>("/g2_command", 100);
//    _g2_pub    = nh.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 100);
    _laser_pub = nh.advertise<sensor_msgs::PointCloud2>("/laser/pointcloud", 10);
    _pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 10);

//    _g2_sub    = nh.subscribe("/joint_states", 10, &G2Control::publish_tf_body_to_laser, this);
    _laser_sub = nh.subscribe("/laser/scan", 10, &G2Control::publish_point_cloud, this);

    _init = true;
    _tf_publishing = false;
//    _g2_angle = 0.0;
}

/*void G2Control::publish_tf_body_to_laser(const sensor_msgs::JointState &msg){
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(msg.position[0] + M_PI, 0.0, 0.0);
  transform.setRotation(q);
  _br.sendTransform(tf::StampedTransform(transform, msg.header.stamp, "chasis_link", "hokuyo_link"));
}

/*
void G2Control::publish_g2_angle(const ros::TimerEvent& event) {

    // To configure param dynamically
    _nh_ptr->getParam("/g2_control/spin_freq", _spin_freq);

    _time_now = ros::Time::now().toSec();
    _time_delta = _init ? 0.01 : _time_now - _time_old;
    _time_old = _time_now;
    _init = false;

    if(fabs(_time_delta - 0.01) > 0.015 || fabs(_time_delta - 0.01) < 0.005 ) {
        ROS_INFO("delta_t not valid");
        _time_delta = 0.01;
    }

    _g2_angle = M_PI*_spin_freq*_time_delta;

//    if(_g2_angle > 0x8ffffffffffffffe) {
//        ROS_ERROR("g2 angle exceeds numeric limit, stopping now.");
//        return;
//    }

//    ROS_INFO("angle: %0.4f; dt: %0.4f", _g2_angle, _time_delta);
//    std_msgs::Float64 msg;
//    msg.data = _g2_angle;
//    _g2_pub.publish(msg);

    gazebo_msgs::LinkState msg;
    msg.link_name = "dji::hokuyo_link";
    tf::Quaternion q;
    q.setRPY(_g2_angle, 0.0, 0.0);
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.reference_frame = "dji::hokuyo_link";
    _g2_pub.publish(msg);


}
*/

void G2Control::publish_point_cloud(const sensor_msgs::LaserScan &msg) {

    ROS_INFO_ONCE("Got laser scan");
    /*if(!_listener.waitForTransform(
        msg.header.frame_id,
        "/chasis_base_link",
        msg.header.stamp + ros::Duration().fromSec(msg.ranges.size()*msg.time_increment),
        ros::Duration(0.02))){
        ROS_WARN("transform");
        return;
    }*/

    if(!_listener.waitForTransform(
        msg.header.frame_id,
        "/gazebo_world",
        msg.header.stamp + ros::Duration().fromSec(msg.ranges.size()*msg.time_increment),
        ros::Duration(0.02))){
        ROS_WARN("transform");
        return;
    }

    sensor_msgs::PointCloud pc_msg;
    _projector.transformLaserScanToPointCloud("/gazebo_world",msg,
              pc_msg,_listener);

    sensor_msgs::PointCloud2 pc2_msg;
    sensor_msgs::convertPointCloudToPointCloud2(pc_msg, pc2_msg);

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(pc2_msg,pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud_ptr = pcl_cloud.makeShared();

    // voxelgrid down sampling
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud_ptr);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*pcl_cloud_filtered_ptr);


    pcl::PointCloud<pcl::PointXYZ> map_raw;
    map_raw = map + *pcl_cloud_filtered_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    map_ptr = map_raw.makeShared();
    pcl::VoxelGrid<pcl::PointXYZ> map_sor;
    map_sor.setInputCloud(map_ptr);
    map_sor.setLeafSize(0.01f, 0.01f, 0.01f);
    map_sor.filter(*map_filtered_ptr);

    map = *map_filtered_ptr;
    // truncating in a range
    /*
    double rangeLim = 25.0;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits (rangeLim, rangeLim);
    pass.filter(pcl_cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits (-rangeLim, rangeLim);
    pass.filter(pcl_cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits (-rangeLim, rangeLim);
    pass.filter(pcl_cloud);

    // truncate in a bounding range
    pcl::ConditionOr<pcl::PointXYZ>::Ptr rangeCond (new pcl::ConditionOr<pcl::PointXYZ> ());
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, -0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, -0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, 0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, -0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, 0.5)));

    pcl::ConditionalRemoval<pcl::PointXYZ> condRem;
    condRem.setCondition((rangeCond));
    condRem.setInputCloud(pcl_cloud_ptr);
    condRem.setKeepOrganized(true);
    condRem.filter(pcl_cloud);
    */

    sensor_msgs::PointCloud2 pc2_msg_out;
    sensor_msgs::PointCloud2 map_msg_out;
    pcl::toROSMsg(pcl_cloud, pc2_msg_out);
    pcl::toROSMsg(map, map_msg_out);
    map_msg_out.header.frame_id = "/gazebo_world";
    _laser_pub.publish(pc2_msg_out);
    _pcl_pub.publish(map_msg_out);
}
