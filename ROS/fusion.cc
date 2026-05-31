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

#include "fusion.h"
#include "visual/visual_drawer_rviz.h"

#include "common/gpstime.h"
#include "common/logging.h"
#include "common/misc.h"
#include "visual/visual_frame.h"

#include <absl/time/clock.h>
#include <absl/time/time.h>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <filesystem>
#include <functional>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <thread>
#include <utility>
#include <yaml-cpp/yaml.h>

std::atomic<bool> global_finished = false;

namespace {

double stampToSec(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

template <typename MessageT>
std::shared_ptr<MessageT> deserializeBagMessage(const rosbag2_storage::SerializedBagMessage &bag_message) {
    rclcpp::SerializedMessage serialized_msg(*bag_message.serialized_data);
    auto message = std::make_shared<MessageT>();
    rclcpp::Serialization<MessageT> serialization;
    serialization.deserialize_message(&serialized_msg, message.get());
    return message;
}

} // namespace

Fusion::Fusion(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)) {}

void Fusion::setFinished() {
    if (vins_) {
        vins_->setFinished();
    }
}

void Fusion::run(const string &config_file) {
    // 加载配置
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file " << config_file << std::endl;
        return;
    }
    auto outputpath        = config["outputpath"].as<string>();
    auto is_make_outputdir = config["is_make_outputdir"].as<bool>();

    // message topic
    string imu_topic, image_topic, lidar_topic;
    imu_topic   = config["ros"]["imu_topic"].as<string>();
    image_topic = config["ros"]["image_topic"].as<string>();
    lidar_topic = config["ros"]["lidar_topic"].as<string>();
    if (config["ros"]["use_compressed_image"]) {
        use_compressed_image_ = config["ros"]["use_compressed_image"].as<bool>();
    }

    // 读取ROS包
    bool is_read_bag = config["ros"]["is_read_bag"].as<bool>();
    string bag_file  = config["ros"]["bag_file"].as<string>();
    // ROS具有高优先级, 重置配置文件的ROS读取配置
    is_read_bag       = node_->declare_parameter<bool>("is_read_bag", is_read_bag);
    string bag_file_ros = node_->declare_parameter<string>("bagfile", bag_file);
    if (!bag_file_ros.empty()) {
        bag_file = bag_file_ros;
    }

    // 如果文件夹不存在, 尝试创建
    if (!std::filesystem::is_directory(outputpath)) {
        std::filesystem::create_directory(outputpath);
    }
    if (!std::filesystem::is_directory(outputpath)) {
        std::cout << "Failed to open outputpath" << std::endl;
        return;
    }

    if (is_make_outputdir) {
        absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
        absl::StrAppendFormat(&outputpath, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                              cs.minute(), cs.second());
        std::filesystem::create_directory(outputpath);
    }
    // 设置Log输出路径
    FLAGS_log_dir = outputpath;

    double imu_data_rate = config["imu"]["imudatarate"].as<double>();
    imu_data_dt_         = 1.0 / imu_data_rate;

    if (config["imu"]["imu_orientation"]) {
        imu_orientation_ = config["imu"]["imu_orientation"].as<std::string>();
        LOGI << "IMU orientation: " << imu_orientation_;
    } else {
        LOGI << "IMU orientation: Default (FRD)";
    }

    // LiDAR参数
    is_use_lidar_depth_      = config["visual"]["is_use_lidar_depth"].as<bool>();
    lidar_type_              = config["lidar"]["lidar_type"].as<int>();
    int scan_line            = config["lidar"]["scan_line"].as<int>();
    double nearest_distance  = config["lidar"]["nearest_distance"].as<double>();
    double farthest_distance = config["lidar"]["farthest_distance"].as<double>();
    double frame_rate        = config["lidar"]["frame_rate"].as<double>();

    if (is_use_lidar_depth_) {
        // lidar数据转换对象
        lidar_converter_ = std::make_shared<LidarConverter>(frame_rate, scan_line, nearest_distance, farthest_distance);
    }

    // ROS传感器数据
    if (is_use_lidar_depth_) {
        if (lidar_type_ == Livox) {
            LOGI << "Process livox lidar messages";
        } else if (lidar_type_ == Velodyne) {
            LOGI << "Process velodyne lidar messages";
        } else if (lidar_type_ == Ouster) {
            LOGI << "Process ouster lidar messages";
        } else if (lidar_type_ == Hesai) {
            LOGI << "Process hesai lidar messages";
        } else {
            LOGE << "Unsupported lidar type";
            return;
        }
    }

    // 创建VINS
    VisualDrawerRviz::Ptr visual_drawer = nullptr;
    bool is_use_visualization_          = config["is_use_visualization"].as<bool>();
    if (is_use_visualization_) {
        visual_drawer = std::make_shared<VisualDrawerRviz>(node_);
    }
    vins_ = std::make_shared<VINS>(config_file, outputpath, visual_drawer);

    // check is initialized
    if (!vins_->isRunning()) {
        LOGE << "Fusion ROS terminate";
        return;
    }

    // 处理ROS数据
    if (is_read_bag) {
        LOGI << "Start to read ROS bag file";
        processRead(imu_topic, image_topic, lidar_topic, bag_file);
        LOGI << "Finish to read ROS bag file";

        // 结束处理
        global_finished = true;
    } else {
        processSubscribe(imu_topic, image_topic, lidar_topic);
    }
}

void Fusion::processSubscribe(const string &imu_topic, const string &image_topic, const string &lidar_topic) {
    //  IMU
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS().keep_last(200),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) { imuCallback(msg); });

    // Visual image
    if (use_compressed_image_) {
        compressed_image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
            image_topic, rclcpp::SensorDataQoS().keep_last(20),
            [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) { imageCallback(msg); });
    } else {
        image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            image_topic, rclcpp::SensorDataQoS().keep_last(20),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(msg); });
    }

    // Lidar
    if (is_use_lidar_depth_) {
        if (lidar_type_ == Livox) {
            livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lidar_topic, rclcpp::SensorDataQoS().keep_last(10),
                [this](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) { livoxCallback(msg); });
        } else {
            lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                lidar_topic, rclcpp::SensorDataQoS().keep_last(10),
                [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { pointCloudCallback(msg); });
        }
    }

    LOGI << "Waiting ROS message...";

    // Enter message loopback
    rclcpp::spin(node_);
}

void Fusion::processRead(const string &imu_topic, const string &image_topic, const string &lidar_topic,
                         const string &bagfile) {
    // 消息列表
    vector<string> topics;
    topics.emplace_back(imu_topic);
    topics.emplace_back(image_topic);
    if (is_use_lidar_depth_) {
        topics.emplace_back(lidar_topic);
    }

    // 遍历ROS包
    rosbag2_cpp::Reader reader;
    reader.open(bagfile);
    rosbag2_storage::StorageFilter filter;
    filter.topics = topics;
    reader.set_filter(filter);

    while (reader.has_next()) {
        auto msg = reader.read_next();

        // 等待数据处理完毕
        while (!global_finished && !vins_->canAddData()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        // 强制退出信号
        if (global_finished) {
            return;
        }

        if (msg->topic_name == imu_topic) {
            auto imumsg = deserializeBagMessage<sensor_msgs::msg::Imu>(*msg);
            imuCallback(imumsg);
        } else if (msg->topic_name == image_topic) {
            if (use_compressed_image_) {
                auto imagemsg = deserializeBagMessage<sensor_msgs::msg::CompressedImage>(*msg);
                imageCallback(imagemsg);
            } else {
                auto imagemsg = deserializeBagMessage<sensor_msgs::msg::Image>(*msg);
                imageCallback(imagemsg);
            }
        } else if (is_use_lidar_depth_ && (msg->topic_name == lidar_topic)) {
            if (lidar_type_ == Livox) {
                auto livox_ptr = deserializeBagMessage<livox_ros_driver2::msg::CustomMsg>(*msg);
                livoxCallback(livox_ptr);
            } else {
                auto points_ptr = deserializeBagMessage<sensor_msgs::msg::PointCloud2>(*msg);
                pointCloudCallback(points_ptr);
            }
        }
    }

    // 等待数据处理结束
    int sec_cnts = 0;
    while (!vins_->isBufferEmpty()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (sec_cnts++ > 20) {
            LOGW << "Waiting vins processing timeout";
            break;
        }
    }
}

void Fusion::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &imumsg) {
    imu_pre_ = imu_;

    // Time convertion
    double unixsecond = stampToSec(imumsg->header.stamp);
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    imu_.time = weeksec;
    // delta time
    imu_.dt = imu_.time - imu_pre_.time;

    // IMU measurements, Front-Right-Down
    if (imu_orientation_ == "RFU") {
        // RFU: X=Right, Y=Front, Z=Up
        // FRD: Front(Y), Right(X), Down(-Z)
        imu_.dtheta[0] = imumsg->angular_velocity.y * imu_.dt;
        imu_.dtheta[1] = imumsg->angular_velocity.x * imu_.dt;
        imu_.dtheta[2] = -imumsg->angular_velocity.z * imu_.dt;
        imu_.dvel[0]   = imumsg->linear_acceleration.y * imu_.dt;
        imu_.dvel[1]   = imumsg->linear_acceleration.x * imu_.dt;
        imu_.dvel[2]   = -imumsg->linear_acceleration.z * imu_.dt;
    } else if (imu_orientation_ == "FLU") {
        // FLU: X=Front, Y=Left, Z=Up
        // FRD: Front(X), Right(-Y), Down(-Z)
        imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
        imu_.dtheta[1] = -imumsg->angular_velocity.y * imu_.dt;
        imu_.dtheta[2] = -imumsg->angular_velocity.z * imu_.dt;
        imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
        imu_.dvel[1]   = -imumsg->linear_acceleration.y * imu_.dt;
        imu_.dvel[2]   = -imumsg->linear_acceleration.z * imu_.dt;
    } else if (imu_orientation_ == "FRD") {
        // FRD: X=Front, Y=Right, Z=Down
        // FRD: Front(X), Right(Y), Down(Z)
        imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
        imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
        imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
        imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
        imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
        imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt;
    } else if (imu_orientation_ == "LBU") {
        // LBU to FRD
        imu_.dtheta[0] = -imumsg->angular_velocity.y * imu_.dt;
        imu_.dtheta[1] = -imumsg->angular_velocity.x * imu_.dt;
        imu_.dtheta[2] = -imumsg->angular_velocity.z * imu_.dt;
        imu_.dvel[0]   = -imumsg->linear_acceleration.y * imu_.dt;
        imu_.dvel[1]   = -imumsg->linear_acceleration.x * imu_.dt;
        imu_.dvel[2]   = -imumsg->linear_acceleration.z * imu_.dt;
    } else {
        // Default to FRD if unknown
        imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
        imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
        imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
        imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
        imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
        imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt;
    }

    // 数据未准备好
    if (imu_pre_.time == 0) {
        return;
    }

    addImuData(imu_);
}

void Fusion::addImuData(const IMU &imu) {
    imu_buffer_.push(imu);
    while (!imu_buffer_.empty()) {
        auto temp = imu_buffer_.front();

        // add new IMU
        if (vins_->addNewImu(temp)) {
            imu_buffer_.pop();
        } else {
            // thread lock failed, try next time
            break;
        }
    }
}

void Fusion::imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &imagemsg) {
    cv::Mat image;

    vector<uint8_t> buffer(imagemsg->data);
    image = cv::imdecode(buffer, cv::IMREAD_COLOR);

    // Time convertion
    double unixsecond = stampToSec(imagemsg->header.stamp);
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    // Add new Image to VINS
    auto latest_frame = visual::VisualFrame::createFrame(weeksec, image);

    visual_frame_buffer_.push(latest_frame);
    while (!visual_frame_buffer_.empty()) {
        auto frame = visual_frame_buffer_.front();
        if (vins_->addNewVisualFrame(frame)) {
            visual_frame_buffer_.pop();
        } else {
            break;
        }
    }
}

void Fusion::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &imagemsg) {
    cv::Mat image;

    // 构造图像
    if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
        // Gray
        image = cv::Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::BGR8) {
        // BGR8

        // OpenCV color format is RGB
        image = cv::Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
    }

    // Time convertion
    double unixsecond = stampToSec(imagemsg->header.stamp);
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    // Add new Image to VINS
    auto latest_frame = visual::VisualFrame::createFrame(weeksec, image);

    visual_frame_buffer_.push(latest_frame);
    while (!visual_frame_buffer_.empty()) {
        auto frame = visual_frame_buffer_.front();
        if (vins_->addNewVisualFrame(frame)) {
            visual_frame_buffer_.pop();
        } else {
            break;
        }
    }
}

void Fusion::livoxCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &lidarmsg) {
    PointCloudCustomPtr pointcloud_ds  = PointCloudCustomPtr(new PointCloudCustom);
    PointCloudCustomPtr pointcloud_raw = PointCloudCustomPtr(new PointCloudCustom);
    double start, end;

    lidar_converter_->livoxPointCloudConvertion(lidarmsg, pointcloud_raw, pointcloud_ds, start, end, true);

    // 激光深度增强点云
    vins_->addNewPointCloud(pointcloud_raw);
}

void Fusion::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidarmsg) {
    PointCloudCustomPtr pointcloud = PointCloudCustomPtr(new PointCloudCustom);
    double start = 0, end = 0;

    if (lidar_type_ == Velodyne) {
        lidar_converter_->velodynePointCloudConvertion(lidarmsg, pointcloud, start, end, true);
    } else if (lidar_type_ == Ouster) {
        lidar_converter_->ousterPointCloudConvertion(lidarmsg, pointcloud, start, end, true);
    } else if (lidar_type_ == Hesai) {
        lidar_converter_->hesaiPointCloudConvertion(lidarmsg, pointcloud, start, end, true);
    }

    // 激光深度增强点云
    vins_->addNewPointCloud(pointcloud);
}
