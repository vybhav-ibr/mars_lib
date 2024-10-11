






#include <gmock/gmock.h>
#include <mars/sensors/uwb/uwb_measurement_type.h>
#include <mars/sensors/uwb/uwb_sensor_state_type.h>
#include <mars/sensors/uwb/uwb_sensor_class.h>
#include <mars/sensors/uwb/uwb_sensor_data.h>
#include <mars/type_definitions/base_states.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>

class mars_uwb_sensor_test : public testing::Test
{
public:
};

TEST_F(mars_uwb_sensor_test, CTOR_UWB_SENSOR)
{
  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::UwbSensorClass uwb_sensor("Uwb", core_states_sptr);
}

TEST_F(mars_uwb_sensor_test, UWB_SENSOR_MEASUREMENT)
{
  double timestamp=100000;
  int id=1;
  double range =0.0;

  mars::UwbMeasurementType measurement(timestamp,id,range);

  std::cout << "Range: " << measurement.range << std::endl;
  std::cout << "ID: " << measurement.id << std::endl;
}

TEST_F(mars_uwb_sensor_test, UWB_SENSOR_INIT)
{
  double timestamp=100000;
  int id=1;
  double range =0.0;

  mars::UwbMeasurementType measurement(timestamp,id,range);


  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::UwbSensorClass uwb_sensor("Uwb", core_states_sptr);

  EXPECT_DEATH(uwb_sensor.Initialize(1, std::make_shared<mars::UwbMeasurementType>(measurement),
                                      std::make_shared<mars::CoreType>()),
               "");
}

TEST_F(mars_uwb_sensor_test, UWB_UPDATE)
{
  double timestamp=100000;
  int id=1;
  double range =0.0;

  mars::UwbMeasurementType measurement(timestamp,id,range);


  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::UwbSensorClass uwb_sensor("Uwb", core_states_sptr);

  mars::CoreStateType prior_core_state;
  mars::BufferDataType prior_sensor_buffer_data;
  Eigen::Matrix<double, prior_core_state.size_error_ + 1, prior_core_state.size_error_ + 1> prior_cov;
  prior_cov.setIdentity();

  // TODO no update without init
  //  int timestamp = 1;
  //  mars::BufferDataType test =
  //      pose_sensor.CalcUpdate(timestamp, std::make_shared<mars::PoseMeasurementType>(measurement), prior_core_state,
  //                             prior_sensor_buffer_data.sensor_, prior_cov);
}