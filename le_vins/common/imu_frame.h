/*
 * Copyright (C) 2026 Wuhan University
 *
 *     Author : Leran Fu
 *    Contact : flrsgg@whu.edu.cn
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

#ifndef IMU_FRAME_H
#define IMU_FRAME_H

#include <Eigen/Geometry>

#include <string>

class ImuFrame {

public:
    static Eigen::Matrix3d rawToFrdRotation(const std::string &orientation) {
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

        if (orientation == "RFU") {
            // RFU raw: X=Right, Y=Front, Z=Up. FRD: X=Front, Y=Right, Z=Down.
            rotation << 0, 1, 0, 1, 0, 0, 0, 0, -1;
        } else if (orientation == "FLU") {
            // FLU raw: X=Front, Y=Left, Z=Up.
            rotation << 1, 0, 0, 0, -1, 0, 0, 0, -1;
        } else if (orientation == "LBU") {
            // Keep this consistent with the ROS IMU conversion in Fusion::imuCallback.
            rotation << 0, -1, 0, -1, 0, 0, 0, 0, -1;
        }

        return rotation;
    }

    static void transformCameraExtrinsicToFrd(const std::string &orientation, Eigen::Quaterniond &q_b_c,
                                              Eigen::Vector3d &t_b_c) {
        const Eigen::Matrix3d r_frd_raw = rawToFrdRotation(orientation);
        q_b_c = Eigen::Quaterniond(r_frd_raw * q_b_c.normalized().toRotationMatrix()).normalized();
        t_b_c = r_frd_raw * t_b_c;
    }
};

#endif // IMU_FRAME_H
