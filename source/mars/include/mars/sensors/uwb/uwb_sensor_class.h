// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef UWBSENSORCLASS_H
#define UWBSENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/general_functions/utils.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/sensors/uwb/uwb_measurement_type.h>
#include <mars/sensors/uwb/uwb_sensor_state_type.h>
#include <mars/sensors/uwb/uwb_sensor_data.h>

#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/core_state_type.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace mars
{
using UwbSensorData = BindSensorData<UwbSensorStateType>;

class UwbSensorClass : public UpdateSensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  UwbSensorClass(const std::string& name, std::shared_ptr<CoreState> core_states)
  {
    name_ = name;
    core_states_ = std::move(core_states);
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    // chi2
    chi2_.set_dof(1);

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  virtual ~UwbSensorClass() = default;

  UwbSensorStateType get_state(const std::shared_ptr<void>& sensor_data)
  {
    UwbSensorData data = *static_cast<UwbSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(const std::shared_ptr<void>& sensor_data)
  {
    UwbSensorData data = *static_cast<UwbSensorData*>(sensor_data.get());
    return data.get_full_cov();
  }

  void set_initial_calib(std::shared_ptr<void> calibration)
  {
    initial_calib_ = calibration;
    initial_calib_provided_ = true;
  }

  BufferDataType Initialize(const Time& timestamp, std::shared_ptr<void> /*sensor_data*/,
                            std::shared_ptr<CoreType> latest_core_data)
  {
    //UwbMeasurementType measurement = *static_cast<UwbMeasurementType*>(sensor_data.get());

    UwbSensorData sensor_state;
    std::string calibration_type;
    this->initial_calib_provided_=true;
    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      UwbSensorData calib = *static_cast<UwbSensorData*>(initial_calib_.get());

      sensor_state.state_ = calib.state_;
      sensor_state.sensor_cov_ = calib.sensor_cov_;
    }
    else
    {
      calibration_type = "Auto";
      std::cout << "UWB Auto calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<UwbSensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << sensor_state.state_.p_ip_.transpose() << " ]" << std::endl;
    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    UwbMeasurementType* meas = static_cast<UwbMeasurementType*>(measurement.get());
    UwbSensorData* prior_sensor_data = static_cast<UwbSensorData*>(latest_sensor_data.get());
    
    // Decompose sensor measurement
    double p_meas = meas->range;
    int anchor_id=meas->id;
    AnchorData* anchor = new AnchorData(2);

    try{
      delete anchor;
      anchor=new AnchorData(anchor_id);
    }catch (const std::out_of_range &oor) {
    std::cout<<"[UWB Update] No anchor found for the given measurement ID %d"<< anchor_id;
    exit(EXIT_FAILURE);
    }

    // Extract sensor state
    UwbSensorStateType prior_sensor_state(prior_sensor_data->state_);

    //R_meas is noise
    //const Eigen::Matrix<double, 1, 1> R_meas = pow(0.5,2)*Eigen::MatrixXd::Identity(1, 1);
    Eigen::Matrix<double, 1, 1> R_meas(pow(0.5,2));

    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    // Calculate the measurement jacobian H
    const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d P_ip = prior_sensor_state.p_ip_;

    Eigen::Matrix3d R_GtoI = R_wi;
    Eigen::Vector3d p_IinG = P_wi;
    Eigen::Vector3d p_IinU = P_ip;

    // Allocate our residual, corrected measurement and Jacobians and predicted measurment (H_x_y = derivative of x wrt y)
    Eigen::MatrixXd H_n = Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd H_z_I = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd H_I = Eigen::MatrixXd::Zero(1, 6);
    Eigen::MatrixXd H_z_cal = Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd H_z_anc = Eigen::MatrixXd::Zero(1, 5);

    Eigen::MatrixXd H_Ir= Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd H_Ip= Eigen::MatrixXd::Zero(1, 3);
    // Position

    const Eigen::Vector3d Hp_vwi = Eigen::Vector3d::Zero();
    const Eigen::Vector3d Hp_bw = Eigen::Vector3d::Zero();
    const Eigen::Vector3d Hp_ba = Eigen::Vector3d::Zero();

    double res;
    res = p_meas -
      ((1 + anchor->dist_bias) * ((anchor->p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm()) + anchor->const_bias);

    H_n.noalias() = ((anchor->p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).transpose()) /
      (anchor->p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm();
    H_z_I.block(0, 0, 3, 3).noalias() =  -R_wi * Utils::Skew(P_ip);
    H_z_I.block(0, 3, 3, 3).noalias() = -Eigen::Matrix3d::Identity();
    H_I.noalias() = (1 + anchor->dist_bias) * H_n * H_z_I;

    H_Ir.noalias() =  (1 + anchor->dist_bias) * H_n * (-R_wi * Utils::Skew(P_ip));
    H_Ip.noalias() =  (1 + anchor->dist_bias) * H_n * (-Eigen::Matrix3d::Identity());
    // Compute Jacobian wrt calibration state
    H_z_cal.noalias() = (1 + anchor->dist_bias) * H_n * R_GtoI.transpose();

    // Compute Jacobian wrt anchor (p_AinU, const_bias, dist_bias)
    H_z_anc.block(0, 0, 1, 3).noalias() = (1 + anchor->dist_bias) * H_n * R_GtoI.transpose();
    H_z_anc(0, 3) = 1;
    H_z_anc(0, 4) = (anchor->p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm();

    // compute jacobian wrt bias
    int width_of_jacobian=  H_Ip.cols() + Hp_vwi.transpose().cols() + H_Ir.cols() + Hp_bw.transpose().cols() + Hp_ba.transpose().cols() + H_z_anc.cols();

    Eigen::MatrixXd H(1, H_Ip.cols() + Hp_vwi.transpose().cols() + H_Ir.cols() + Hp_bw.transpose().cols() + Hp_ba.transpose().cols() + H_z_cal.cols());//+ H_z_cal.cols());

    H << H_Ip, Hp_vwi.transpose(), H_Ir, Hp_bw.transpose(), Hp_ba.transpose(), H_z_cal;//, H_z_anc ;
// convert linear vector to 3d matrices with matrix multiplication, 
// (ask if possible) ignore distance and const bias, use the notation used in mars
// use spearte jaconbians for callibaration state, anchor and stack at the end
// enquire the dimesion of the noise jacobian and convert the single value to a 3*3 matrix if required   
    
    Eigen::Matrix<double, 1, 1> residual_(res);
    //residual_ = res_p(0);

    // Perform EKF calculations
    mars::Ekf ekf(H, R_meas, residual_, P);
    const Eigen::MatrixXd correction = ekf.CalculateCorrection(&chi2_);
    assert(correction.size() == size_of_full_error_state * 1);

    // Perform Chi2 test
    if (!chi2_.passed_ && chi2_.do_test_)
    {
      chi2_.PrintReport(name_);
      return false;
    }

    Eigen::MatrixXd P_updated = ekf.CalculateCovUpdate();
    assert(P_updated.size() == size_of_full_error_state * size_of_full_error_state);
    P_updated = Utils::EnforceMatrixSymmetry(P_updated);

    // Apply Core Correction
    CoreStateVector core_correction = correction.block(0, 0, CoreStateType::size_error_, 1);
    CoreStateType corrected_core_state = CoreStateType::ApplyCorrection(prior_core_state, core_correction);

    // Apply Sensor Correction
    const Eigen::MatrixXd sensor_correction = correction.block(size_of_core_state, 0, size_of_sensor_state, 1);
    const UwbSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data

    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<UwbSensorData> sensor_data(std::make_shared<UwbSensorData>());
    sensor_data->set_cov(P_updated);
    sensor_data->state_ = corrected_sensor_state;

    BufferDataType state_entry(std::make_shared<CoreType>(core_data), sensor_data);

    if (const_ref_to_nav_)
    {
      // corrected_sensor_data.ref_to_nav = prior_ref_to_nav;
    }
    else
    {
      // TODO(chb) also estimate ref to nav
    }

    *new_state_data = state_entry;

    return true;
  }

  UwbSensorStateType ApplyCorrection(const UwbSensorStateType& prior_sensor_state,
                                        const Eigen::MatrixXd& correction)
  {
    

    UwbSensorStateType corrected_sensor_state;
    corrected_sensor_state.p_ip_ = prior_sensor_state.p_ip_ + correction.block(0, 0, 3, 1);
    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // UwbSENSORCLASS_H
