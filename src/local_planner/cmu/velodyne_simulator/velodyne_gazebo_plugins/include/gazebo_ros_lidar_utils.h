//
// Created by ou on 2021/6/20.
//

#ifndef VELODYNE_GAZEBO_PLUGINS_GAZEBO_ROS_LIDAR_UTILS_H
#define VELODYNE_GAZEBO_PLUGINS_GAZEBO_ROS_LIDAR_UTILS_H

// boost stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string.hpp>

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// ros messages stuff
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

// Gazebo stuff
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>


namespace gazebo {
    class GazeboRosTriggeredLidar;

    class GazeboRosLidarUtils {
    public:
        GazeboRosLidarUtils();

        ~GazeboRosLidarUtils();

        void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    private:
        void LoadRos();

        void Init();

    protected:
        gazebo::transport::NodePtr gazebo_node_;
        gazebo::transport::SubscriberPtr gazebo_sub_;
        sensors::SensorPtr parentSensor_;
        physics::WorldPtr world_;
        sdf::ElementPtr sdf_;
        std::string lidar_name_;
    protected:
        bool was_active_;
        int lidar_connect_count_;
        boost::mutex lidar_connect_count_lock_;

        void LidarConnect();

        void LidarDisconnect();

        virtual void OnScan(const ConstLaserScanStampedPtr &_msg);

    protected:
        ros::NodeHandle *nh_;
        ros::Publisher pointcloud2_pub_;

        sensor_msgs::PointCloud2 pointcloud2_msg_;

        std::string lidar_topic_name_;
        std::string robot_namespace_;
        std::string tf_prefix_;
        std::string frame_name_;

        double min_range_;
        double max_range_;
    protected:
        virtual void TriggerLidar();

        virtual bool CanTriggerLidar();

    private:
        void TriggerLidarInternal(const std_msgs::Empty::ConstPtr &dummy);

        ros::Subscriber trigger_subscriber_;
        std::string trigger_topic_name_;

    protected:
        boost::mutex lock_;
        ros::CallbackQueue lidar_queue_;

        void lidarQueueThread();

        boost::thread callback_queue_thread_;

    protected:
        bool initialized_;

    public:
        template<class T>
        inline bool ReadSdfTag(std::string _tag_name, T &_output, T _defalut) {
            bool exists = this->sdf_->HasElement(_tag_name);
            _output = exists ? this->sdf_->GetElement(_tag_name)->Get<T>() : _defalut;
            ROS_INFO_STREAM((exists ? "Read" : "Miss") << " Tag <" << _tag_name << ">: "
                                                       << (exists ? "default to " : "") << _output);
            return exists;
        }

        inline bool GetSdfTag(std::string _tag_name, sdf::ElementPtr &element) {
            bool exists = this->sdf_->HasElement(_tag_name);
            element = exists ? this->sdf_->GetElement(_tag_name) : NULL;
            ROS_INFO_STREAM((exists ? "Get" : "Miss") << " Tag <" << _tag_name << ">");
            return exists;
        }
        inline bool GetSdfTag(sdf::ElementPtr _sdf,std::string _tag_name, sdf::ElementPtr &element) {
            bool exists = _sdf->HasElement(_tag_name);
            element = exists ? _sdf->GetElement(_tag_name) : NULL;
            ROS_INFO_STREAM((exists ? "Get" : "Miss") << " Tag <" << _tag_name << ">");
            return exists;
        }
        friend class GazeboRosTriggeredLidar;
    };
}


#endif //VELODYNE_GAZEBO_PLUGINS_GAZEBO_ROS_LIDAR_UTILS_H
