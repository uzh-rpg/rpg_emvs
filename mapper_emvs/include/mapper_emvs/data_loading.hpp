#pragma once

#include <string>
#include <map>
#include <ros/time.h>
#include <mapper_emvs/geometry_utils.hpp>
#include <dvs_msgs/EventArray.h>
#include <camera_info_manager/camera_info_manager.h>

namespace data_loading {

void parse_rosbag(const std::string &rosbag,
                  std::vector<dvs_msgs::Event>& events_,
                  std::map<ros::Time, geometry_utils::Transformation>& poses_,
                  sensor_msgs::CameraInfo& camera_info_msg,
                  const std::string& event_topic,
                  const std::string& camera_info_topic,
                  const std::string& pose_topic,
                  const double tmin,
                  const double tmax);

} // namespace data_loading
