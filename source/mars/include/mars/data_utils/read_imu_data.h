// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef READ_IMU_DATA_H
#define READ_IMU_DATA_H

#include <mars/data_utils/read_csv.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>
#include <vector>

namespace mars
{
class ReadImuData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReadImuData(std::vector<BufferEntryType>* data_out, std::shared_ptr<SensorAbsClass> sensor,
              const std::string& file_path, const double& time_offset = 0)
  {
    std::vector<std::string> expect_entry = { "t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z" };

    CsvDataType csv_data;
    ReadCsv(&csv_data, file_path);

    unsigned long number_of_datapoints = csv_data["t"].size();
    data_out->resize(number_of_datapoints);

    for (size_t k = 0; k < number_of_datapoints; k++)
    {
      Time time = csv_data["t"][k] + time_offset;

      Eigen::Vector3d linear_acceleration(csv_data["a_x"][k], csv_data["a_y"][k], csv_data["a_z"][k]);
      Eigen::Vector3d angular_velocity(csv_data["w_x"][k], csv_data["w_y"][k], csv_data["w_z"][k]);

      BufferDataType data;
      data.set_measurement(std::make_shared<IMUMeasurementType>(linear_acceleration, angular_velocity));

      BufferEntryType current_entry(time, data, sensor);
      data_out->at(k) = current_entry;
    }
  }
};
}  // namespace mars

#endif  // READ_IMU_DATA_H