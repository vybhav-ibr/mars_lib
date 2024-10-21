// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef UWBMEASUREMENTTYPE_H
#define UWBMEASUREMENTTYPE_H

#include <mars/sensors/measurement_base_class.h>
#include <mars/sensors/uwb/uwb_sensor_data.h>
#include <Eigen/Dense>
#include <utility>

namespace mars
{
class UwbMeasurementType : public BaseMeas
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //double timestamp;
  int id;
  double range;

  // Eigen::Vector3d position_;
  // Eigen::VectorXd<double> dv_pairs_;               ///< distance validity pairs
  // //Eigen::Quaternion<double> orientation_;  ///< Quaternion [w x y z]
  UwbMeasurementType() = default;

  UwbMeasurementType(int id_, double range_):id(id_),range(range_)
  {
  }

  // UwbMeasurementType(Eigen::VectorXd<double> dv_pairs_, const Eigen::Vector3d& position)
  //   : position_(std::move(position))
  // {
  // }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "id, ";
    os << "range, ";

    return os.str();
  }

   std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << "," << id;
    os << "," << range;
    

    return os.str();
  }
};
}  // namespace mars
#endif  // UWBMEASUREMENTTYPE_H
