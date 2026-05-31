/*
 * Copyright (C) 2024 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef FUSION_H
#define FUSION_H

#include "lidar/lidar_converter.h"

#include "common/types.h"
#include "le_vins/le_vins.h"

#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <atomic>
#include <memory>
#include <queue>

extern std::atomic<bool> global_finished;

class Fusion {

public:
    explicit Fusion(rclcpp::Node::SharedPtr node);

    void run(const string &config_file);

    void setFinished();

private:
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &imumsg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &imagemsg);
    void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &imagemsg);
    void livoxCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &lidarmsg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidarmsg);

    void addImuData(const IMU &imu);

    void processSubscribe(const string &imu_topic, const string &image_topic, const string &lidar_topic);
    void processRead(const string &imu_topic, const string &image_topic, const string &lidar_topic,
                     const string &bagfile);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<VINS> vins_;

    bool is_use_lidar_depth_{false};

    IMU imu_{.time = 0}, imu_pre_{.time = 0};
    double imu_data_dt_{0.005};
    std::string imu_orientation_{"FRD"}; // Default to FRD

    bool use_compressed_image_{false};

    std::queue<IMU> imu_buffer_;
    std::queue<VisualFrame::Ptr> visual_frame_buffer_;

    LidarConverter::Ptr lidar_converter_;
    int lidar_type_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
};

#endif // FUSION_ROS_H
