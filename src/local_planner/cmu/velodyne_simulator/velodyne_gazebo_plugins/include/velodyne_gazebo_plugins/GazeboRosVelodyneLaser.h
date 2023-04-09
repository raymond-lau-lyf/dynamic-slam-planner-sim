/**
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef GAZEBO_ROS_VELODYNE_LASER_H_
#define GAZEBO_ROS_VELODYNE_LASER_H_

// Use the same source code for CPU and GPU plugins
#ifndef GAZEBO_GPU_RAY
#define GAZEBO_GPU_RAY 0
#endif

// Custom Callback Queue
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#if GAZEBO_GPU_RAY
#include <gazebo/plugins/GpuRayPlugin.hh>
#else
#include <gazebo/plugins/RayPlugin.hh>
#endif

#include <boost/algorithm/string/trim.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#if GAZEBO_GPU_RAY
#define GazeboRosVelodyneLaser GazeboRosVelodyneGpuLaser
#define RayPlugin GpuRayPlugin
#define RaySensorPtr GpuRaySensorPtr
#endif

namespace gazebo {

class GazeboRosVelodyneLaser : public RayPlugin   {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosVelodyneLaser(); // NOLINT
    /// \brief Destructor
    public: ~GazeboRosVelodyneLaser(); // NOLINT
    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf); // NOLINT
    /// \brief Subscribe on-demand
    private: void ConnectCb(); // NOLINT
    /// \brief The parent ray sensor
    private: sensors::RaySensorPtr parent_ray_sensor_; // NOLINT
    /// \brief Pointer to ROS node
    private: ros::NodeHandle* nh_; // NOLINT
    /// \brief ROS publisher
    private: ros::Publisher pub_; // NOLINT
    /// \brief topic name
    private: std::string topic_name_; // NOLINT
    /// \brief frame transform name, should match link name
    private: std::string frame_name_; // NOLINT
    /// \brief organize cloud
    private: bool organize_cloud_; // NOLINT
    /// \brief the intensity beneath which points will be filtered
    private: double min_intensity_; // NOLINT
    /// \brief Minimum range to publish
    private: double min_range_; // NOLINT
    /// \brief Maximum range to publish
 private: double max_range_; // NOLINT
    /// \brief Gaussian noise
 private: double gaussian_noise_; // NOLINT
    /// \brief Gaussian noise generator
 private: static double gaussianKernel(double mu, double sigma) {  // NOLINT
      // using Box-Muller transform to generate two independent standard normally distributed normal variables
      // see wikipedia
      double U = (double)rand() / (double)RAND_MAX; // normalized uniform random variable  // NOLINT
      double V = (double)rand() / (double)RAND_MAX; // normalized uniform random variable // NOLINT
      return sigma * (sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V)) + mu;
    }

    /// \brief A mutex to lock access
 private: boost::mutex lock_; // NOLINT
    /// \brief For setting ROS name space
 private: std::string robot_namespace_; // NOLINT
    // Custom Callback Queue
 private: ros::CallbackQueue laser_queue_; // NOLINT
 private: void laserQueueThread(); // NOLINT
 private: boost::thread callback_laser_queue_thread_; // NOLINT
    // Subscribe to gazebo laserscan
 private: gazebo::transport::NodePtr gazebo_node_; // NOLINT
 private: gazebo::transport::SubscriberPtr sub_; // NOLINT
 private: void OnScan(const ConstLaserScanStampedPtr &_msg); // NOLINT
};

} // namespace gazebo

#endif /* GAZEBO_ROS_VELODYNE_LASER_H_ */

