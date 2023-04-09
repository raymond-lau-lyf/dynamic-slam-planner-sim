/**
 * @copyright Copyright (c) 2022
 * 
 */
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <time.h>

using namespace std;  //NOLINT

const double PI = 3.1415926;

#define PLOTPATHSET 1

string pathFolder;  //NOLINT

double vehicleLength = 0.6;
double vehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true; /* 是否是能两个方向行走 */
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = false;
bool checkObstacle = true;
bool checkRotObstacle = false;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre = 0.1;
double costScore = 0.02;
bool useCost = false;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.25;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre =
    90.0; /* 应该是限制路径的采样角度，在机器人延goal方向的正负dirThre内采样 */
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75; /* 0.75 */
double pathScaleStep = 0.25;
bool pathScaleBySpeed = true;
double minPathRange = 1.0; /* 1.0 */
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double joyToCheckObstacleDelay = 5.0;
double goalClearRange = 0.5;
double goalX = 0; /* way_point话题输入的goal */
double goalY = 0; /* way_point话题输入的goal */

float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0; /* goal相对于vehicle的角度 */

const int pathNum = 343;
const int groupNum = 7; /* 一个group中path的数量 */
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161; /* 还不错到是干啥的 */
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(
    new pcl::PointCloud<pcl::PointXYZI>()); /* 降采样的在机器人周围的地形点云 */
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudStack[laserCloudStackNum]; /* 用于累积的点云stack */
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr
    startPaths[groupNum]; /* 存放每个group的路径 */
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum]; /* 存放每个path */
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(
    new pcl::PointCloud<pcl::PointXYZI>());
#endif

int pathList[pathNum] = {0};         /* 记录每个path的group  */
float endDirPathList[pathNum] = {0}; /* 记录每个path的end direction */
int clearPathList[36 * pathNum] = {0}; /* 应该是表示每个path点，有多少个 */
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {
    0}; /* 表示每个start_path的分数 */
std::vector<int>
    correspondences[gridVoxelNum]; /* 记录一个grid voxel有哪些paths经过 */

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0,
      vehicleYaw = 0; /*里程计输入的机器人的姿态*/
float vehicleX = 0, vehicleY = 0, vehicleZ = 0; /*里程计输入的机器人的位置*/

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom) {
  odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX +
             sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX -
             cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
}

/**
 * @brief 以world为坐标系，用于进行地形识别
 *
 * @param laserCloud2
 */
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2) {
  if (!useTerrainAnalysis) {
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                       (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

    newLaserCloud = true;
  }
}

void terrainCloudHandler(
    const sensor_msgs::PointCloud2ConstPtr& terrainCloud2) {
  if (useTerrainAnalysis) {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                       (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange &&
          (point.intensity > obstacleHeightThre || useCost)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
  }
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy) {
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;

  if (joySpeed > 0) {
    joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
    if (joy->axes[4] < 0) joyDir *= -1;
  }

  if (joy->axes[4] < 0 && !twoWayDrive) joySpeed = 0;

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    checkObstacle = true;
  } else {
    checkObstacle = false;
  }
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal) {
  goalX = goal->point.x;
  goalY = goal->point.y;
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

void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary) {
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / terrainVoxelSize) + 1; // NOLINT
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + // NOLINT
                  (1.0 - float(pointID) / float(pointNum)) * point2.x; // NOLINT
        point.y = float(pointID) / float(pointNum) * point1.y + // NOLINT
                  (1.0 - float(pointID) / float(pointNum)) * point2.y; // NOLINT
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < pointPerPathThre; j++) {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}

void addedObstaclesHandler(
    const sensor_msgs::PointCloud2ConstPtr& addedObstacles2) {
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++) {
    addedObstacles->points[i].intensity = 200.0;
  }
}

void checkObstacleHandler(const std_msgs::Bool::ConstPtr& checkObs) {
  double checkObsTime = ros::Time::now().toSec();

  if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
    checkObstacle = checkObs->data;
  }
}

int readPlyHeader(FILE* filePtr) {
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

/**
 * @brief  从path file中读取预计算的路径到startPaths中
 *
 */
void readStartPaths() {
  string fileName = pathFolder + "/startPaths.ply";

  FILE* filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
/**
 * @brief 从path file中读取预计算的路径到paths中
 *
 */
void readPaths() {
  string fileName = pathFolder + "/paths.ply";

  FILE* filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

/**
 * @brief 读取每个path的信息
 *
 */
void readPathList() {
  string fileName = pathFolder + "/pathList.ply";

  FILE* filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    printf("\nIncorrect path number, exit.\n\n");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  fclose(filePtr);
}

/**
 * @brief 记录每个grid voxel的path id
 *
 */
void readCorrespondences() {
  string fileName = pathFolder + "/correspondences.txt";

  FILE* filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        printf("\nError reading input files, exit.\n\n");
        exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 &&
            pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pathFolder", pathFolder);
  nhPrivate.getParam("vehicleLength", vehicleLength);
  nhPrivate.getParam("vehicleWidth", vehicleWidth);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis);
  nhPrivate.getParam("checkObstacle", checkObstacle);
  nhPrivate.getParam("checkRotObstacle", checkRotObstacle);
  nhPrivate.getParam("adjacentRange", adjacentRange);
  nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
  nhPrivate.getParam("groundHeightThre", groundHeightThre);
  nhPrivate.getParam("costHeightThre", costHeightThre);
  nhPrivate.getParam("costScore", costScore);
  nhPrivate.getParam("useCost", useCost);
  nhPrivate.getParam("pointPerPathThre", pointPerPathThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("dirWeight", dirWeight);
  nhPrivate.getParam("dirThre", dirThre);
  nhPrivate.getParam("dirToVehicle", dirToVehicle);
  nhPrivate.getParam("pathScale", pathScale);
  nhPrivate.getParam("minPathScale", minPathScale);
  nhPrivate.getParam("pathScaleStep", pathScaleStep);
  nhPrivate.getParam("pathScaleBySpeed", pathScaleBySpeed);
  nhPrivate.getParam("minPathRange", minPathRange);
  nhPrivate.getParam("pathRangeStep", pathRangeStep);
  nhPrivate.getParam("pathRangeBySpeed", pathRangeBySpeed);
  nhPrivate.getParam("pathCropByGoal", pathCropByGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
  nhPrivate.getParam("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  nhPrivate.getParam("goalClearRange", goalClearRange);
  nhPrivate.getParam("goalX", goalX);
  nhPrivate.getParam("goalY", goalY);

  ros::Subscriber subOdometry =
      nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/registered_scan", 5, laserCloudHandler);

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/terrain_map", 5, terrainCloudHandler);

  ros::Subscriber subJoystick =
      nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

  ros::Subscriber subGoal =
      nh.subscribe<geometry_msgs::PointStamped>("/way_point", 5, goalHandler);

  ros::Subscriber subSpeed =
      nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

  ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped>(
      "/navigation_boundary", 5, boundaryHandler);

  ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2>(
      "/added_obstacles", 5, addedObstaclesHandler);

  ros::Subscriber subCheckObstacle =
      nh.subscribe<std_msgs::Bool>("/check_obstacle", 5, checkObstacleHandler);

  ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 5);
  nav_msgs::Path path; /* 要发布的path，相对于机器人坐标系 */

#if PLOTPATHSET == 1
  ros::Publisher pubFreePaths =
      nh.advertise<sensor_msgs::PointCloud2>("/free_paths", 2);
#endif

  // ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
  // ("/stacked_scans", 2);

  printf("\nReading path files.\n");

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;  // 限制角速度
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
#if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
#endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize,
                               terrainVoxelSize);

  readStartPaths();  // 从path file中读取预计算的路径到startPaths中
#if PLOTPATHSET == 1
  readPaths();  // 从path file中读取预计算的路径到paths中
#endif
  readPathList();         // 读取每个path的信息
  readCorrespondences();  // 记录每个grid voxel的path id

  printf("\nInitialization complete.\n\n");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserCloud ||
        newTerrainCloud /* 有新的地形图进行，就要一直进行规划 */) {
      if (newLaserCloud) {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud) {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;  // 使用地形点云作为规划使用的点云
      }

      float sinVehicleRoll = sin(vehicleRoll);
      float cosVehicleRoll = cos(vehicleRoll);
      float sinVehiclePitch = sin(vehiclePitch);
      float cosVehiclePitch = cos(vehiclePitch);
      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);

      pcl::PointXYZI point;
      plannerCloudCrop->clear();
      int plannerCloudSize = plannerCloud->points.size();
      // 将terrainCloudDwz转到vehicle坐标系下
      for (int i = 0; i < plannerCloudSize; i++) {
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange &&
            ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
          plannerCloudCrop->push_back(point);
        }
      }

      // 先不管
      int boundaryCloudSize = boundaryCloud->points.size();
      for (int i = 0; i < boundaryCloudSize; i++) {
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw +
                   (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw +
                   (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      // 先不管
      int addedObstaclesSize = addedObstacles->points.size();
      for (int i = 0; i < addedObstaclesSize; i++) {
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw +
                   (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw +
                   (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      float pathRange = adjacentRange;                             // 4.25
      if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;  // = 4.25*1
      if (pathRange < minPathRange) pathRange = minPathRange;      // = 1
      float relativeGoalDis = adjacentRange; /* goal距离vehicle的距离 */

      if (autonomyMode) {
        float relativeGoalX =
            ((goalX - vehicleX) * cosVehicleYaw +
             (goalY - vehicleY) * sinVehicleYaw); /* goal相对于vehicle的坐标 */
        float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw +
                               (goalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis =
            sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

        if (!twoWayDrive) {  // 展示角度范围为-90～90
          if (joyDir > 90.0)
            joyDir = 90.0;
          else if (joyDir < -90.0)
            joyDir = -90.0;
        }
      }

      bool pathFound = false;
      float defPathScale = pathScale;                             // 1.25
      if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;  // = 1.25 * 1
      if (pathScale < minPathScale) pathScale = minPathScale;

      while (pathScale >= minPathScale && pathRange >= minPathRange) {
        for (int i = 0; i < 36 * pathNum; i++) {
          clearPathList[i] = 0;
          pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        float diameter =
            sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 +
                 vehicleWidth / 2.0 * vehicleWidth / 2.0); /* 机器人半径 */
        float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize;
             i++) {  // 遍历每个地形点云的点
          float x = plannerCloudCrop->points[i].x / pathScale;
          float y = plannerCloudCrop->points[i].y / pathScale;
          float h = plannerCloudCrop->points[i]
                        .intensity; /* intensity表示通行的cost */
          float dis = sqrt(x * x + y * y);

          if (dis < pathRange / pathScale &&
              (dis <= (relativeGoalDis + goalClearRange) / pathScale ||
               !pathCropByGoal) &&
              checkObstacle) {  // 太远的点不考虑
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }
              if ((angDiff > dirThre && !dirToVehicle) ||
                  (fabs(10.0 * rotDir - 180.0) > dirThre &&
                   fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre &&
                    360.0 - 10.0 * rotDir > dirThre) &&
                   fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
              }

              float x2 =
                  cos(rotAng) * x + sin(rotAng) * y;  // 旋转x,y到 旋转角度下
              float y2 = -sin(rotAng) * x + cos(rotAng) * y;

              float scaleY = x2 / gridVoxelOffsetX +
                             searchRadius / gridVoxelOffsetY *
                                 (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);  // NOLINT
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize); // NOLINT
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 &&
                  indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                int blockedPathByVoxelNum =
                    correspondences[ind].size();  // 对于经过这个grid的所有path
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  if (h > obstacleHeightThre /* 表示这个grid是有障碍物的 */ ||
                      !useTerrainAnalysis) {  //
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                  } else {
                    if (pathPenaltyList[pathNum * rotDir +
                                        correspondences[ind][j]] < h &&
                        h > groundHeightThre) {
                      pathPenaltyList[pathNum * rotDir +
                                      correspondences[ind][j]] = h;  //
                    }
                  }
                }
              }
            }
          }

          if (dis < diameter / pathScale &&
              (fabs(x) > vehicleLength / pathScale / 2.0 ||
               fabs(y) > vehicleWidth / pathScale / 2.0) &&
              (h > obstacleHeightThre || !useTerrainAnalysis) &&
              checkRotObstacle) {
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset)
                minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0)
                minObsAngCW = angObs + angOffset - 180.0;
            } else {
              if (minObsAngCW < angObs + angOffset)
                minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0 + angObs - angOffset)
                minObsAngCCW = 180.0 + angObs - angOffset;
            }
          }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        for ( int i = 0; i < 36 * pathNum; i++) { // NOLINT
          int rotDir = int(i / pathNum);  // NOLINT
          float angDiff = fabs(
              joyDir - (10.0 * rotDir -
                        180.0));  // 表示这个采样的方向 与 目标向量之间的夹角
          if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;
          }
          if ((angDiff > dirThre /* 这个条件限制了采样范围 */ &&
               !dirToVehicle /* false */) ||
              (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 &&
               dirToVehicle) ||
              ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) &&
               fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
          }

          if (clearPathList[i] < pointPerPathThre) {
            float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
            if (penaltyScore < costScore) penaltyScore = costScore;

            float dirDiff =
                fabs(joyDir - endDirPathList[i % pathNum] -
                     (10.0 * rotDir - 180.0)); /* 改path与目标向量的偏差角度 */
            if (dirDiff > 360.0) {
              dirDiff -= 360.0;
            }
            if (dirDiff > 180.0) {
              dirDiff = 360.0 - dirDiff;
            }

            float rotDirW;
            if (rotDir < 18)
              rotDirW = fabs(fabs(rotDir - 9) + 1);
            else
              rotDirW = fabs(fabs(rotDir - 27) + 1);
            float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW *
                          rotDirW * rotDirW * rotDirW * penaltyScore;
            if (score > 0) {
              clearPathPerGroupScore[groupNum * rotDir +
                                     pathList[i % pathNum]] +=
                  score;  // 路径i对应的group，加一分
            }
          }
        }

        float maxScore = 0;
        int selectedGroupID = -1; /* 应该是规划中选择的group id */
        for (int i = 0; i < 36 * groupNum;
             i++) {  // 遍历每一个group，选择score最大的group
          int rotDir =
              int(i / groupNum);  // 在[0,35]之间取值，代表预计算路径旋转的方向  // NOLINT
          float rotAng =
              (10.0 * rotDir - 180.0) * PI /
              180;  // 代表路径的角度， - 180.0应该是代表机器人的正前方
          float rotDeg = 10.0 * rotDir;         // 代表旋转的角度
          if (rotDeg > 180.0) rotDeg -= 360.0;  // 位于[-180,180]
          if (maxScore < clearPathPerGroupScore[i] &&
              ((rotAng * 180.0 / PI > minObsAngCW &&
                rotAng * 180.0 / PI < minObsAngCCW) ||
               (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) ||
               !checkRotObstacle)) {
            maxScore = clearPathPerGroupScore
                [i];  // 重点：36*groupNum个path组合的得分是怎么计算的
            selectedGroupID = i;
          }
        }

        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum);  // NOLINT
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

          selectedGroupID = selectedGroupID % groupNum;
          int selectedPathLength = startPaths[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            if (dis <= pathRange / pathScale &&
                dis <= relativeGoalDis / pathScale) {
              path.poses[i].pose.position.x =
                  pathScale * (cos(rotAng) * x - sin(rotAng) * y);
              path.poses[i].pose.position.y =
                  pathScale * (sin(rotAng) * x + cos(rotAng) * y);
              path.poses[i].pose.position.z = pathScale * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = ros::Time().fromSec(odomTime);
          path.header.frame_id = "/vehicle";
          pubPath.publish(path);

#if PLOTPATHSET == 1
          freePaths->clear();
          for (int i = 0; i < 36 * pathNum; i++) {
            int rotDir = int(i / pathNum); // NOLINT
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0) {
              angDiff = 360.0 - angDiff;
            }
            if ((angDiff > dirThre && !dirToVehicle) ||
                (fabs(10.0 * rotDir - 180.0) > dirThre &&
                 fabs(joyDir) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) &&
                 fabs(joyDir) > 90.0 && dirToVehicle) ||
                !((rotAng * 180.0 / PI > minObsAngCW &&
                   rotAng * 180.0 / PI < minObsAngCCW) ||
                  (rotDeg > minObsAngCW && rotDeg < minObsAngCCW &&
                   twoWayDrive) ||
                  !checkRotObstacle)) {
              continue;
            }

            if (clearPathList[i] < pointPerPathThre) {  // free path
              int freePathLength = paths[i % pathNum]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                point = paths[i % pathNum]->points[j];

                float x = point.x;
                float y = point.y;
                float z = point.z;

                float dis = sqrt(x * x + y * y);
                if (dis <= pathRange / pathScale &&
                    (dis <= (relativeGoalDis + goalClearRange) / pathScale ||
                     !pathCropByGoal)) {
                  point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                  point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                  point.z = pathScale * z;
                  point.intensity = 1.0;

                  freePaths->push_back(point);
                }
              }
            }
          }

          sensor_msgs::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = ros::Time().fromSec(odomTime);
          freePaths2.header.frame_id = "/vehicle";
          pubFreePaths.publish(freePaths2);
#endif
        }

        if (selectedGroupID < 0) {
          if (pathScale >= minPathScale + pathScaleStep) {
            pathScale -= pathScaleStep;
            pathRange = adjacentRange * pathScale / defPathScale;
          } else {
            pathRange -= pathRangeStep;
          }
        } else {
          pathFound = true;
          break;
        }
      }
      pathScale = defPathScale;

      if (!pathFound) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "/vehicle";
        pubPath.publish(path);

#if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = ros::Time().fromSec(odomTime);
        freePaths2.header.frame_id = "/vehicle";
        pubFreePaths.publish(freePaths2);
#endif
      }

      /*sensor_msgs::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
      plannerCloud2.header.stamp = ros::Time().fromSec(odomTime);
      plannerCloud2.header.frame_id = "/vehicle";
      pubLaserCloud.publish(plannerCloud2);*/
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
