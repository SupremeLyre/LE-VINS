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

#include "visual_drawer_rviz.h"

#include "common/logging.h"
#include "common/rotation.h"

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <utility>

VisualDrawerRviz::VisualDrawerRviz(rclcpp::Node::SharedPtr node)
    : is_finished_(false)
    , isframerdy_(false)
    , ismaprdy_(false)
    , node_(std::move(node)) {

    frame_id_      = "world";
    body_frame_id_ = "body";

    pose_pub_           = node_->create_publisher<nav_msgs::msg::Odometry>("/visual/pose", rclcpp::QoS(2));
    path_pub_           = node_->create_publisher<nav_msgs::msg::Path>("/visual/path", rclcpp::QoS(2));
    track_image_pub_    = node_->create_publisher<sensor_msgs::msg::Image>("/visual/tracking", rclcpp::QoS(2));
    raw_image_pub_      = node_->create_publisher<sensor_msgs::msg::Image>("/visual/image", rclcpp::QoS(2));
    fixed_points_pub_   = node_->create_publisher<sensor_msgs::msg::PointCloud>("/visual/fixed", rclcpp::QoS(2));
    current_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("/visual/current", rclcpp::QoS(2));
    tf_broadcaster_     = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

void VisualDrawerRviz::setFinished() {
    is_finished_ = true;
    update_sem_.notify_one();
}

void VisualDrawerRviz::run() {

    LOGI << "Visual drawer thread is started";
    while (!is_finished_) {
        // 等待绘图更新信号
        std::unique_lock<std::mutex> lock(update_mutex_);
        update_sem_.wait(lock);

        // 发布跟踪的图像
        if (isframerdy_) {
            publishTrackingImage();

            isframerdy_ = false;
        }

        // 发布轨迹和地图点
        if (ismaprdy_) {
            publishOdometry();

            publishMapPoints();

            ismaprdy_ = false;
        }
    }
    LOGI << "Visual drawer thread is exited";
}

void VisualDrawerRviz::updateFrame(visual::VisualFrame::Ptr frame, const cv::Mat &undistorted_image) {
    std::unique_lock<std::mutex> lock(image_mutex_);

    frame_ = frame;
    if (!undistorted_image.empty()) {
        undistorted_image.copyTo(undistorted_image_);
    } else {
        undistorted_image_ = cv::Mat();
    }

    isframerdy_ = true;
    update_sem_.notify_one();
}

void VisualDrawerRviz::updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                              vector<visual::MapPointType> mappoint_type) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_map_     = std::move(map);
    pts2d_matched_ = std::move(matched);
    mappoint_type_ = std::move(mappoint_type);
}

void VisualDrawerRviz::updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_ref_ = std::move(ref);
    pts2d_cur_ = std::move(cur);
}

void VisualDrawerRviz::publishTrackingImage() {
    std::unique_lock<std::mutex> lock(image_mutex_);

    frame_->rawImage().copyTo(raw_image_);
    if (!undistorted_image_.empty()) {
        undistorted_image_.copyTo(track_image_);
    } else {
        frame_->image().copyTo(track_image_);
    }

    cv::Mat drawed;
    drawTrackingImage(track_image_, drawed);

    // OpenCV RGB format is saved as BGR

    // Tracked image
    sensor_msgs::msg::Image image;
    image.header.stamp    = node_->now();
    image.header.frame_id = frame_id_;
    image.encoding        = sensor_msgs::image_encodings::BGR8;
    image.height          = drawed.rows;
    image.width           = drawed.cols;

    size_t size = image.height * image.width * 3;
    image.step  = image.width * 3;
    image.data.resize(size);
    memcpy(image.data.data(), drawed.data, size);

    track_image_pub_->publish(image);

    // Raw image
    image.header.stamp    = node_->now();
    image.header.frame_id = frame_id_;
    image.height          = raw_image_.rows;
    image.width           = raw_image_.cols;

    if (raw_image_.channels() == 1) {
        image.encoding = sensor_msgs::image_encodings::MONO8;

        size       = image.height * image.width;
        image.step = image.width;
    } else if (raw_image_.channels() == 3) {
        image.encoding = sensor_msgs::image_encodings::BGR8;

        size       = image.height * image.width * 3;
        image.step = image.width * 3;
    }
    image.data.resize(size);
    memcpy(image.data.data(), raw_image_.data, size);

    raw_image_pub_->publish(image);
}

void VisualDrawerRviz::publishMapPoints() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = node_->now();

    // 发布窗口内的路标点
    sensor_msgs::msg::PointCloud current_pointcloud;

    current_pointcloud.header.stamp    = stamp;
    current_pointcloud.header.frame_id = frame_id_;

    // 获取当前点云
    for (const auto &current : current_mappoints_) {
        geometry_msgs::msg::Point32 point;
        point.x = static_cast<float>(current.x());
        point.y = static_cast<float>(current.y());
        point.z = static_cast<float>(current.z());

        current_pointcloud.points.push_back(point);
    }
    current_points_pub_->publish(current_pointcloud);

    // 发布新的地图点
    sensor_msgs::msg::PointCloud fixed_pointcloud;

    fixed_pointcloud.header.stamp    = stamp;
    fixed_pointcloud.header.frame_id = frame_id_;

    for (const auto &pts : fixed_mappoints_) {
        geometry_msgs::msg::Point32 point;
        point.x = static_cast<float>(pts.x());
        point.y = static_cast<float>(pts.y());
        point.z = static_cast<float>(pts.z());

        fixed_pointcloud.points.push_back(point);
    }
    fixed_points_pub_->publish(fixed_pointcloud);

    fixed_mappoints_.clear();
}

void VisualDrawerRviz::publishOdometry() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    nav_msgs::msg::Odometry odometry;

    auto quaternion = Rotation::matrix2quaternion(pose_.R);
    auto stamp      = node_->now();

    // Odometry
    odometry.header.stamp            = stamp;
    odometry.header.frame_id         = frame_id_;
    odometry.child_frame_id          = body_frame_id_;
    odometry.pose.pose.position.x    = pose_.t.x();
    odometry.pose.pose.position.y    = pose_.t.y();
    odometry.pose.pose.position.z    = pose_.t.z();
    odometry.pose.pose.orientation.x = quaternion.x();
    odometry.pose.pose.orientation.y = quaternion.y();
    odometry.pose.pose.orientation.z = quaternion.z();
    odometry.pose.pose.orientation.w = quaternion.w();
    pose_pub_->publish(odometry);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp            = stamp;
    transform.header.frame_id         = frame_id_;
    transform.child_frame_id          = body_frame_id_;
    transform.transform.translation.x = pose_.t.x();
    transform.transform.translation.y = pose_.t.y();
    transform.transform.translation.z = pose_.t.z();
    transform.transform.rotation      = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    // Path
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp    = stamp;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.pose            = odometry.pose.pose;

    path_.header.stamp    = stamp;
    path_.header.frame_id = frame_id_;
    path_.poses.push_back(pose_stamped);
    path_pub_->publish(path_);
}

void VisualDrawerRviz::updateCurrentMappoints(vector<Vector3d> points) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    current_mappoints_ = std::move(points);
}

void VisualDrawerRviz::updateNewFixedMappoints(vector<Vector3d> points) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    fixed_mappoints_ = std::move(points);
}

void VisualDrawerRviz::updateMap(const Pose &pose) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    pose_ = pose;

    ismaprdy_ = true;
    update_sem_.notify_one();
}
