/**
 * @copyright Copyright (c) 2022
 * 
 */
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <time.h>

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;    /* 0.5 */
double yawRateGain = 7.5;     /* 7.5 */
double stopYawRateGain = 7.5; /* 7.5 */
double maxYawRate = 45.0;     /* 90.0 */
double maxSpeed = 1.0;
double maxAccel = 1.0; /* 2.5 */
double switchTimeThre = 1.0;
double dirDiffThre = 0.1; /* 0.1，角度的控制精度 */
double stopDisThre = 0.2; /* 0.2，距离的控制精度 */
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0; /* 可以通过话题让路径快速停下来，当探索完成时使用 */

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

nav_msgs::Path path; /* 输入的path，相对于机器人坐标系 */

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn) {
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX +
             sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX -
             cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 ||
       fabs(pitch) > inclThre * PI / 180.0) &&
      useInclToStop) {  // 跳过
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 ||
       fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) &&
      useInclRateToSlow) {  // 跳过
    slowInitTime = odomIn->header.stamp.toSec();
  }
}

void pathHandler(const nav_msgs::Path::ConstPtr& pathIn) {
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  // 记录最新的机器人位姿
  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy) {
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed) {
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay &&
      joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::Int8::ConstPtr& stop) {
  safetyStop = stop->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("lookAheadDis", lookAheadDis);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("switchTimeThre", switchTimeThre);
  nhPrivate.getParam("dirDiffThre", dirDiffThre);
  nhPrivate.getParam("stopDisThre", stopDisThre);
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
  nhPrivate.getParam("inclRateThre", inclRateThre);
  nhPrivate.getParam("slowRate1", slowRate1);
  nhPrivate.getParam("slowRate2", slowRate2);
  nhPrivate.getParam("slowTime1", slowTime1);
  nhPrivate.getParam("slowTime2", slowTime2);
  nhPrivate.getParam("useInclToStop", useInclToStop);
  nhPrivate.getParam("inclThre", inclThre);
  nhPrivate.getParam("stopTime", stopTime);
  nhPrivate.getParam("noRotAtStop", noRotAtStop);
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);

  ros::Subscriber subOdom =
      nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odomHandler);

  ros::Subscriber subPath =
      nh.subscribe<nav_msgs::Path>("/path", 5, pathHandler);

  ros::Subscriber subJoystick =
      nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

  ros::Subscriber subSpeed =
      nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

  ros::Subscriber subStop =
      nh.subscribe<std_msgs::Int8>("/stop", 5, stopHandler);

  // ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped>
  // ("/cmd_vel", 5);
  ros::Publisher pubSpeed_no_stamp =
      nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "/vehicle";

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (pathInit) {
      float vehicleXRel =
          cos(vehicleYawRec) * (vehicleX - vehicleXRec) +
          sin(vehicleYawRec) *
              (vehicleY -
               vehicleYRec); /* 机器人当前位置相对于机器人初始位置时的坐标 */
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) +
                          cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX +
                          endDisY * endDisY); /* path最后一个点到机器人的距离 */

      float disX, disY, dis;
      while (pathPointID <
             pathSize - 1) {  // 选择距离起点lookAheadDis处的一个点
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI)
        dirDiff -= 2 * PI;  // 限制在-pi 到 pi
      else if (dirDiff < -PI)
        dirDiff += 2 * PI;
      if (dirDiff > PI)
        dirDiff -= 2 * PI;
      else if (dirDiff < -PI)
        dirDiff += 2 * PI;

      if (twoWayDrive) {
        double time = ros::Time::now().toSec();
        if (fabs(dirDiff) > PI / 2 && navFwd &&
            time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd &&
                   time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {  // 跳过
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      // note: 角速度控制策略
      // 比例控制
      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0)
        vehicleYawRate = -stopYawRateGain * dirDiff;
      else
        vehicleYawRate = -yawRateGain * dirDiff;

      // 限制角速度大小vehicleYawRate
      if (vehicleYawRate > maxYawRate * PI / 180.0)
        vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0)
        vehicleYawRate = -maxYawRate * PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode) {  // 跳过
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {  // 跳过
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0)
        joySpeed3 *= slowRate1;  //跳过 //NOLINT
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 &&
               slowInitTime > 0)
        joySpeed3 *= slowRate2;

      // note： 速度控制的策略
      if (fabs(dirDiff) < dirDiffThre /* 5.7度 */ &&
          dis > stopDisThre) {  // 当角度控制的差不多时，就只控制线速度
        if (vehicleSpeed < joySpeed3)
          vehicleSpeed +=
              maxAccel /
              100.0;  // 线速度的变化每次是根据加速度来变化的，符合动力学
        else if (vehicleSpeed > joySpeed3)
          vehicleSpeed -= maxAccel / 100.0;  // 对线速度做限制
      } else {  // 这种情况让线速度趋向于零，也是先调整
        if (vehicleSpeed > 0)
          vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0)
          vehicleSpeed += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {  // 跳过
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ros::Time().fromSec(odomTime);
        if (fabs(vehicleSpeed) <= maxAccel / 100.0)
          cmd_vel.twist.linear.x = 0;
        else
          cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        // pubSpeed.publish(cmd_vel);
        pubSpeed_no_stamp.publish(cmd_vel.twist);

        pubSkipCount =
            pubSkipNum;  // 每隔selectedGroupID+1次发布一次速度，这样速度的频率就是100/(pubSkipNum+1)
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
