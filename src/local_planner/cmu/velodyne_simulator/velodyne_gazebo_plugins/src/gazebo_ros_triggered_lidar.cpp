//
// Created by ou on 2021/6/20.
//
#include "velodyne_gazebo_plugins/gazebo_ros_triggered_lidar.h"
#include <gazebo_plugins/gazebo_ros_utils.h>

using namespace std;
namespace gazebo {
    GZ_REGISTER_SENSOR_PLUGIN(GazeboRosTriggeredLidar)

    void GazeboRosTriggeredLidar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
        ROS_WARN_STREAM("-------------------");
        ROS_INFO_STREAM("model: " << GetModelName(_parent));
        ROS_INFO_STREAM("plugin: " << _sdf->Get<string>("name") << STR_Gpu_BLOCK);

        RayPlugin::Load(_parent, _sdf);
        ROS_WARN_STREAM("RayPlugin::Load finished");
        GazeboRosLidarUtils::Load(_parent, _sdf);

        this->parentSensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

        sdf::ElementPtr vDistribution, sector;
        if (GetSdfTag("vDistribution", vDistribution)) {
            auto verticalCount = this->parentSensor_->VerticalRangeCount();
            double angleUnit = (this->parentSensor_->VerticalAngleMax() -
                                this->parentSensor_->VerticalAngleMin()).Degree() / verticalCount;
            map_ = vector<bool>(verticalCount, false);
            stringstream info("\n");
            for (int j = 0, num_line = 0; GetSdfTag(vDistribution, "sector" + to_string(j), sector); ++j) {
                gzmsg << "reading sector" + to_string(j) + ": " << sector->GetName() << endl;
                Sector_t sector_obj;
                sector->GetAttribute("begin")->Get(sector_obj.angle[0]);
                sector->GetAttribute("end")->Get(sector_obj.angle[1]);
                sector->GetAttribute("lines")->Get(sector_obj.count);
                sector_obj.step = (sector_obj.angle[0] - sector_obj.angle[1]) / sector_obj.count / angleUnit;
                sector_obj.pixel = sector_obj.step * angleUnit;
                sectors_.push_back(sector_obj);

                auto offset = this->parentSensor_->VerticalAngleMin().Degree();
                int sector_begin_quaNum = floorf((sector_obj.angle[0] - offset) / angleUnit),
                        sector_end_quaNum = ceilf((sector_obj.angle[1] - offset) / angleUnit);

                info << "Sector-" + to_string(j) + ": " << endl;
                for (int sec_line_num = 1; sector_begin_quaNum > sector_end_quaNum && sec_line_num < sector_obj.count;
                     sector_begin_quaNum -= sector_obj.step, sec_line_num++) {
                    map_[sector_begin_quaNum] = true;
                    info << ++num_line << " -> [" << offset + sector_begin_quaNum * angleUnit << "] degree" << endl;
                }
            }
            gzmsg << info.str();
        }

        this->totalPoints_ = this->parentSensor_->VerticalRangeCount() * this->parentSensor_->RangeCount();
        this->pointcloud2_msg_.data.reserve(this->totalPoints_ * this->pointcloud2_msg_.point_step);
        this->pointcloud2_msg_.data.resize(this->totalPoints_ * this->pointcloud2_msg_.point_step);

        this->SetLidarEnabled(false);

        this->updateConnection = event::Events::ConnectBeforePhysicsUpdate(
                std::bind(&GazeboRosTriggeredLidar::OnUpdate, this));

        ROS_WARN_STREAM("GazeboRosTriggeredLidar::Load finished");
    }

    bool GazeboRosTriggeredLidar::CanTriggerLidar() {
        return true;
    }

    void GazeboRosTriggeredLidar::TriggerLidar() {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (!this->parentSensor_)
            return;
        this->triggered++;
        gzmsg << this->lidar_topic_name_ << " triggered: " << this->triggered << endl;
    }

    void GazeboRosTriggeredLidar::OnUpdate() {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (this->triggered > 0) {
            this->SetLidarEnabled(true);
        }
    }

    void GazeboRosTriggeredLidar::SetLidarEnabled(const bool _enabled) {
        this->parentSensor_->SetActive(_enabled);
        this->parentSensor_->SetUpdateRate(_enabled ? 0.0 : DBL_MIN);
    }

/*    void GazeboRosTriggeredLidar::OnNewLaserScans() {
        /// TODO:或使用Sensor::ConnectUpdated(std::function<void()> _subscriber)
        ROS_INFO("on_new_laser_scan");
//        this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();
//
//        if ((*this->image_connect_count_) > 0)
//        {
//            this->PutCameraData(_image);
//            this->PublishCameraInfo();
//        }
        this->SetLidarEnabled(false);
//
        std::lock_guard<std::mutex> lock(this->mutex);
        this->triggered = std::max(this->triggered - 1, 0);
    }*/
    void GazeboRosTriggeredLidar::OnScan(ConstLaserScanStampedPtr &_gz_msg) {

        static const auto maxRange = this->parentSensor_->RangeMax();
        static const auto minRange = this->parentSensor_->RangeMin();
        static const double MIN_RANGE = std::max(min_range_, minRange);
        static const double MAX_RANGE = std::min(max_range_, maxRange);

        static const auto maxAngle = this->parentSensor_->AngleMax();
        static const auto minAngle = this->parentSensor_->AngleMin();
        static const auto verticalMaxAngle = this->parentSensor_->VerticalAngleMax();
        static const auto verticalMinAngle = this->parentSensor_->VerticalAngleMin();

        static const auto rayCount = this->parentSensor_->RayCount();
        static const auto rangeCount = this->parentSensor_->RangeCount();
        static const auto verticalRayCount = this->parentSensor_->VerticalRayCount();
        static const auto verticalRangeCount = this->parentSensor_->VerticalRangeCount();

        static const double yDiff = maxAngle.Radian() - minAngle.Radian();
        static const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();
        static const double yStep = rangeCount == 0 ? 0 : yDiff / (rangeCount - 1);
        static const double pStep = verticalRangeCount == 0 ? 0 : pDiff / (verticalRangeCount - 1);

        static auto &msg = this->pointcloud2_msg_;


        this->SetLidarEnabled(false);
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->triggered = std::max(this->triggered - 1, 0);
        }

        // convert gazebo msg and clock ros topic
        uint8_t *ptr = msg.data.data();
        for (int i = 0; i < rangeCount; i++) {
            for (int j = 0; j < verticalRangeCount; j++) {
                if (!map_[j])
                    continue;
                double r = _gz_msg->scan().ranges(i + j * rangeCount);
                double intensity = _gz_msg->scan().intensities(i + j * rangeCount);
                // Get angles of ray to get xyz for point
                double yAngle = i * yStep + minAngle.Radian();
                double pAngle = j * pStep + verticalMinAngle.Radian();


                // pAngle is rotated by yAngle:
                if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
                    *((float *) (ptr + 0)) = r * cos(pAngle) * cos(yAngle);
                    *((float *) (ptr + 4)) = r * cos(pAngle) * sin(yAngle);
                    *((float *) (ptr + 8)) = r * sin(pAngle);
                    *((float *) (ptr + 12)) = intensity;
                    ptr += msg.point_step;
                } else {
                    *((float *) (ptr + 0)) = 0;
                    *((float *) (ptr + 4)) = 0;
                    *((float *) (ptr + 8)) = 0;
                    *((float *) (ptr + 12)) = 0;
                    continue;
                }
            }
        }
        msg.header.stamp.sec = this->parentSensor_->LastMeasurementTime().sec;
        msg.header.stamp.nsec = this->parentSensor_->LastMeasurementTime().nsec;
        msg.row_step = ptr - msg.data.data();
        msg.height = 1;
        msg.width = msg.row_step / msg.point_step;
        msg.is_bigendian = false;
        msg.is_dense = true;
        msg.data.resize(msg.row_step);

        this->pointcloud2_pub_.publish(msg);
        msg.data.resize(this->totalPoints_ * msg.point_step);

        gzmsg << this->lidar_topic_name_ << " on scan: " << msg.width << " points" << endl;
    }
}