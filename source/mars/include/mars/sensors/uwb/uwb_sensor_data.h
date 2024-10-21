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
#include <iostream>

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
    std::map<int, Eigen::Vector3d> anchor_map={{1, Eigen::Vector3d(3.876000000000931, -0.059999999997671694, 1.859000000000151)},
                {2, Eigen::Vector3d(0.05, 0.5, 0.0)},
                {3, Eigen::Vector3d(-3.777000000004773, 1.1240000000107102, 1.7789999999999964)},
                {4, Eigen::Vector3d(-2.6870000000082657, 11.045999999972992, 1.8100000000001728)},
                {5, Eigen::Vector3d(1.5740000000048895, 10.826000000000931, 0.08300000000008367)},
                {0, Eigen::Vector3d(5.071999999997206, 10.467000000004191, 1.9100000000000819)},
                {7, Eigen::Vector3d(5.85000000000291, 20.602000000013504, 1.9390000000000782)},
                {8, Eigen::Vector3d(2.0960000000020953, 20.98200000001816, 0.13800000000014734)},
                {9, Eigen::Vector3d(-1.9030000000086147, 21.007999999972526, 1.8949999999999818)},
                {10, Eigen::Vector3d(-1.4690000000002328, 30.988000000012107, 1.8110000000001492)},
                {11, Eigen::Vector3d(2.2569999999948775, 31.21700000000419, 0.1570000000001528)},
                {12, Eigen::Vector3d(6.127999999994063, 30.945999999996275, 0.9800000000000182)}}
                ;

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
            std::cerr << "ID not found in anchor_map: " <<id << std::endl;
        }
    }
};

// Static map initialization outside the class definition

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