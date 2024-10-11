/*
 * UVIO: Ultra Wide-Band aided Visual-Inertial Odometry
 * Copyright (C) 2020-2022 Alessandro Fornasier
 * Copyright (C) 2018-2022 UVIO Contributors
 *
 * Control of Networked Systems, University of Klagenfurt, Austria (alessandro.fornasier@aau.at)
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

#ifndef UWB_SENSOR_DATA_H
#define UWB_SENSOR_DATA_H

#include <unordered_map>
#include <Eigen/Eigen>

namespace mars
{

/**
 * @brief Struct for a single uwb measurement (time, anchor_id (0,1,2,3,...), distance measurement)
 */
struct UwbData {

    /// anchor id and timestamp
    double timestamp;
    int id;

    /// range
    double range =-1;
};

class AnchorData {
public:
    int id;

    /// Fixed anchor (no state update)
    bool fix = false;

    /// Biases (y = d + dist_bias * d + const_bias + noise)
    double const_bias = 0;
    double dist_bias = 0;

    // Static map of anchors
    static std::map<int, Eigen::Vector3d> anchor_map;

    /// Position of the anchor in global coordinates
    Eigen::Vector3d p_AinG;

    // Constructor
    AnchorData(int id) {
        auto it = anchor_map.find(id);
        if (it != anchor_map.end()) {
            this->id = id;   
            fix = true;      
            const_bias = 0;
            dist_bias = 0;
            p_AinG = it->second;  // Set the position from the map
        } else {
            std::cerr << "ID not found in anchor_map" << std::endl;
        }
    }
};

// Static map initialization outside the class definition
std::map<int, Eigen::Vector3d> AnchorData::anchor_map = {
    {1, Eigen::Vector3d(0.5, 0.8, 2.5)},
    {2, Eigen::Vector3d(0.25, 0.568, 2.45)}
};

/**
 * @brief Struct for anchors information (id, anchors)
 
struct AnchorData {

    /// anchor id and timestamp
    double timestamp;
    size_t id;

    /// Fixed anchor (no state update)
    bool valid = false;

    /// Biases (y = d + dist_bias * d + const_bias + noise)
    double const_bias = 0;
    double dist_bias = 0;

    /// p_AinG
    double distance;

    /// covariance of the estimation (5x5)
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(5, 5);
};
*/
}

#endif // UVIO_SENSOR_DATA_H