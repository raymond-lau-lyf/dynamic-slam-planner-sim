/**
 * @copyright Copyright (c) 2022
 * 
 */

#include <cmath>

#include <ros/ros.h> // NOLINT
#include <std_srvs/Empty.h> // NOLINT

#include <tf/transform_broadcaster.h> // NOLINT
#include <nav_msgs/Odometry.h> // NOLINT
#include <nav_msgs/Path.h> // NOLINT
#include <sensor_msgs/PointCloud2.h> // NOLINT

#include <Eigen/Core> // NOLINT
#include <Eigen/Geometry> // NOLINT

#include <pcl/point_cloud.h> // NOLINT
#include <pcl/point_types.h> // NOLINT
#include <pcl_conversions/pcl_conversions.h> // NOLINT

#define PI 3.1415926535897932384626433

static std::string odom_topic = "/omron_ros_wheel/odom"; // NOLINT
static std::string parent_frame = "/odom"; // NOLINT
static std::string child_frame = "/base_link"; // NOLINT
static std::string path_topic = "/path_gt"; // NOLINT
static std::string path_points_topic = "/path_points_gt"; // NOLINT
static bool path_enable = false; // NOLINT
static float path_dist_threshold = 0.1;          // Unit:m, 每隔path_dist_threshold发布一个point // NOLINT
static float path_angle_threshold = 5;          // Unit:agree, 每隔path_angle_threshold发布一个point // NOLINT


ros::Publisher  pub_path, pub_path_points;
nav_msgs::Path path;
sensor_msgs::PointCloud2 path_points;
pcl::PointCloud<pcl::PointXYZI>::Ptr path_points_pcl(new pcl::PointCloud<pcl::PointXYZI>()); // NOLINT

Eigen::Isometry3d nav_msgsPose2Eigen(const geometry_msgs::Pose* _pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond result_r(_pose->orientation.w,_pose->orientation.x, // NOLINT
                                _pose->orientation.y,_pose->orientation.z); // NOLINT
  Eigen::Vector3d result_t(_pose->position.x,_pose->position.y,_pose->position.z); // NOLINT
  result.rotate(result_r);
  result.pretranslate(result_t);
  return result;
}

void odom_callback(const nav_msgs::OdometryConstPtr& odom) {
  static uint count = 1;

  // 发布tf
  static tf::TransformBroadcaster br;
  tf::Transform tf;
  geometry_msgs::Pose odom_pose = odom->pose.pose;

//  tf::Quaternion quat;
//  tf::quaternionMsgToTF(odom_pose.orientation, quat);
//  tf.setRotation(quat);
  tf::poseMsgToTF(odom_pose, tf);
  tf::StampedTransform stamped_tf(tf, odom->header.stamp, parent_frame, child_frame); // NOLINT

  br.sendTransform(stamped_tf);
  if(++count % 100 == 0) {  // NOLINT
    pub_path.publish(path);
    pcl::toROSMsg(*path_points_pcl, path_points);
    path_points.header = path.header;
    pub_path_points.publish( path_points );  // NOLINT
    if( path.poses.size() % 1000 == 0 && path.poses.size() != 0) {  // NOLINT
      ROS_WARN("SIZE in the path:%d", path.poses.size() );  // 实验测试size大于16000时，rviz会崩  // NOLINT
    }
    count = 1;
  }


  // update path
  static bool is_first = true;
  static geometry_msgs::PoseStamped pre_pose;
  if(is_first) { // NOLINT
    pre_pose.header = odom->header;
    pre_pose.pose = odom->pose.pose;
    path.header.frame_id = parent_frame;
    path.header.stamp = odom->header.stamp;
    path.poses.push_back(pre_pose);
    pcl::PointXYZI tmp_way_point;
    tmp_way_point.x = pre_pose.pose.position.x;
    tmp_way_point.y = pre_pose.pose.position.y;
    tmp_way_point.z = pre_pose.pose.position.z;
    tmp_way_point.intensity = path.poses.size();
    path_points_pcl->push_back(tmp_way_point);
    is_first = false;
    return;
  }
  Eigen::Isometry3d pre_pose_eigen = nav_msgsPose2Eigen(&pre_pose.pose);
  Eigen::Isometry3d curr_pose_eigen = nav_msgsPose2Eigen(&odom->pose.pose);
  Eigen::Isometry3d delta_pose_eigin = curr_pose_eigen.inverse() * pre_pose_eigen; // NOLINT
  Eigen::Vector3d delta_euler = delta_pose_eigin.rotation().eulerAngles(2,1,0);  // NOLINT
  Eigen::Vector3d delta_trans = delta_pose_eigin.translation();
  // std::cout << delta_pose_eigin.matrix() << std::endl;
  // std::cout << delta_euler << std::endl;
  // std::cout << delta_trans << std::endl;


  if( std::min( abs( delta_euler[0] ), PI - abs( delta_euler[0] ) ) < path_angle_threshold &&  // NOLINT
      std::min( abs( delta_euler[1] ), PI - abs( delta_euler[1] ) )  < path_angle_threshold &&  // NOLINT
      std::min( abs( delta_euler[2] ), PI - abs( delta_euler[2] ) )  < path_angle_threshold &&  // NOLINT
     delta_trans.norm() < path_dist_threshold )  {
    return;  // NOLINT
  }  else {
    path.header.stamp = odom->header.stamp;
    path.poses.push_back(pre_pose);
    pcl::PointXYZI tmp_way_point;
    tmp_way_point.x = pre_pose.pose.position.x;
    tmp_way_point.y = pre_pose.pose.position.y;
    tmp_way_point.z = pre_pose.pose.position.z;
    tmp_way_point.intensity = path.poses.size();
    path_points_pcl->push_back(tmp_way_point);
    pre_pose.pose = odom->pose.pose;
    pre_pose.header = odom->header;
  }
}

bool clear_path_handler(std_srvs::Empty::Request &rqt, std_srvs::Empty::Response &res) {
  ROS_INFO("The size of current Path is %d.", path.poses.size());
  ROS_INFO(" Path clear finished!");
  path.poses.clear();
  path_points_pcl->clear();
  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "odom2tf");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("odom_topic", odom_topic);
  private_nh.getParam("parent_frame", parent_frame);
  private_nh.getParam("child_frame", child_frame);
  private_nh.getParam("path_topic", path_topic);
  private_nh.getParam("path_points_topic", path_points_topic);
  private_nh.getParam("path_enable", path_enable);
  private_nh.getParam("path_dist_threshold", path_dist_threshold);
  private_nh.getParam("path_angle_threshold", path_angle_threshold);

  std::cout << "Odom topic: " << odom_topic << std::endl;
  std::cout << "Parent frame: " << parent_frame << std::endl;
  std::cout << "Child frame: " << child_frame << std::endl;
  std::cout << "Path topic: " << path_topic << std::endl;
  std::cout << "Path points topic: " << path_points_topic << std::endl;
  std::cout << "path_enable: " << path_enable << std::endl;
  std::cout << "path_dist_threshold: " << path_dist_threshold << std::endl;
  std::cout << "path_angle_threshold: " << path_angle_threshold << std::endl;

  pub_path = nh.advertise<nav_msgs::Path>(path_topic, 10,true); // NOLINT
  pub_path_points = nh.advertise<sensor_msgs::PointCloud2>(path_points_topic, 10,true); // NOLINT
  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odom_callback);
  ros::ServiceServer clear_path_service = nh.advertiseService("clear_path", clear_path_handler); // NOLINT
  ros::spin();

  return 0;
}  // NOLINT