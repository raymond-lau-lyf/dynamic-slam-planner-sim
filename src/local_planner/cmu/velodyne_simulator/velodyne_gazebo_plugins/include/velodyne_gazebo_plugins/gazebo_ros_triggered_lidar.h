//
// Created by ou on 2021/6/20.
//

#ifndef VELODYNE_GAZEBO_PLUGINS_GAZEBO_ROS_TRIGGERED_LIDAR_H
#define VELODYNE_GAZEBO_PLUGINS_GAZEBO_ROS_TRIGGERED_LIDAR_H

#include "gazebo_ros_lidar_utils.h"

#ifndef GAZEBO_GPU_RAY
#define GAZEBO_GPU_RAY 0
#endif

#if GAZEBO_GPU_RAY

#else


#endif
#if GAZEBO_GPU_RAY
#include <gazebo/plugins/GpuRayPlugin.hh>
#define GazeboRosTriggeredLidar GazeboRosTriggeredGpuLidar
#define RayPlugin GpuRayPlugin
#define RaySensorPtr GpuRaySensorPtr
#define RaySensor GpuRaySensor
#define STR_Gpu  "Gpu"
#define STR_GPU_ "GPU "
#define STR_Gpu_BLOCK "(GPU)"
#else

#include <gazebo/plugins/RayPlugin.hh>

#define STR_Gpu  ""
#define STR_GPU_ ""
#define STR_Gpu_BLOCK ""
#endif
typedef struct {
    double angle[2];
    int count;
    double pixel;
    int step;
} Sector_t;
namespace gazebo {
    class GazeboRosTriggeredLidar : public GazeboRosLidarUtils, public RayPlugin {
    public:
        GazeboRosTriggeredLidar() {};

        ~GazeboRosTriggeredLidar() {};

        void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

        void TriggerLidar();

        bool CanTriggerLidar();

        void SetLidarEnabled(const bool _enabled);

        event::ConnectionPtr updateConnection;

    protected:
        void OnUpdate();

//        virtual void OnNewLaserScans();
        void OnScan(const ConstLaserScanStampedPtr &_gz_msg);

        sensors::RaySensorPtr parentSensor_;
        std::vector<Sector_t> sectors_;
        std::vector<bool> map_;
        int triggered = 0;
        std::mutex mutex;
        size_t totalPoints_;
    };
}
#endif //VELODYNE_GAZEBO_PLUGINS_GAZEBO_ROS_TRIGGERED_LIDAR_H
