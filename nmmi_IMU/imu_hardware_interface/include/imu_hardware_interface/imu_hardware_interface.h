/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2019, Centro "E. Piaggio"
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IMU_HARDWARE_INTERFACE_H
#define IMU_HARDWARE_INTERFACE_H

// ROS libraries
#include <pluginlib/class_list_macros.hpp>

// internal libraries
#include <qb_device_hardware_interface/qb_device_hardware_interface.h>
#include <imu_hardware_interface/imu_transmission_interface.h>
#include <nmmi_msgs/nmmi_msgs.h>
#include <nmmi_srvs/nmmi_srvs.h>
#include <eigen3/Eigen/Eigen>

namespace imu_hardware_interface {
/**
 * The \em IMU HardWare interface implements the specific structures to manage the communication with the
 * \em IMU device. It exploits the features provided by the base device-independent hardware interface and the
 * specific transmission interface.
 * \sa qb_device_hardware_interface::qbDeviceHW, IMU_transmission_interface::IMUVirtualTransmission
 */
class IMUHW : public qb_device_hardware_interface::qbDeviceHW {
 public:
  /**
   * Initialize the \p qb_device_hardware_interface::qbDeviceHW with the specific transmission interface and actuator
   * and joint names.
   * \sa imu_transmission_interface::IMUVirtualTransmission
   */
  IMUHW();

  /**
   * Do nothing.
   */
  virtual ~IMUHW();
  
  /**
   * \return The vector of controller joint names.
   */
  std::vector<std::string> getJoints();

  /**
   * Call the base method and nothing more.
   * \param root_nh A NodeHandle in the root of the caller namespace.
   * \param robot_hw_nh A NodeHandle in the namespace from which the RobotHW should read its configuration.
   * \returns \p true on success.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);

  /**
   * Call the base method and nothing more.
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   */
  void read(const ros::Time& time, const ros::Duration& period);

  /**
   * Call the base method and nothing more.
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   */
  void write(const ros::Time& time, const ros::Duration& period);

protected:

  float*   imu_values_;
  int      n_imu_;
  std::vector<uint8_t> ids_;
  bool get_imu_values_;
  bool compute_quaternions_node_;
  bool compute_angles_;
  bool is_reliable_;
  bool custom_read_timeout_;
  int consecutive_failures_;

  std::vector<double> Acc_;
  std::vector<double> Gyro_;
  std::vector<double> Mag_;
  std::vector<double> Quat_;
  std::vector<double> Temp_;
  std::vector<double> Acc_old_;
  std::vector<double> Ext_Quat_;
  ros::Time time_stamp_;


  // Message variables with IMUs table
  nmmi_msgs::ImuTable imu_table_msg_;

  // Publisher variables
  ros::Publisher imuboard_pub_state_;
  ros::Publisher imuboard_pub_acc_;
  ros::Publisher imuboard_pub_gyro_;
  ros::Publisher imuboard_pub_mag_;
  ros::Publisher imuboard_pub_quat_;
  ros::Publisher imuboard_pub_temp_;
  ros::Publisher imuboard_pub_angles_;
      
  void compute_quat_ext(int n, const ros::Duration& period, int read_fail);

  int getIMUvalues();
  
  /**
   * Call the service to initialize the device with parameters from the Communication Handler and wait for the response.
   * If the initializaation succeed, store the device parameters received, e.g. \p n_imu_. 
   * \return \p 0 on success.
   * \sa waitForInitialization()
   */
  int initializeDevice();

  /**
   * Subscribe to all the services advertised by the Communication Handler and wait until all the services are
   * properly advertised.
   * \sa resetServicesAndWait(), waitForServices()
   */
  void initializeIMUServicesAndWait();
  
  /**
   * Construct a \p qb_device_msgs::StateStamped message of the whole device state with the data retrieved during the
   * \p read() and publish it to a namespaced \p "~state" topic.
   * \sa read()
   */
  void publish();

  void quat_to_angles(std::vector<float> q, std::vector<float> &a);

  /**
   * Re-subscribe to all the services advertised by the Communication Handler and wait until all the services are
   * properly advertised. Then re-initialize the device parameters (it is assumed that the this method can be called
   * only if the device was previously initialized), unless otherwise specified.
   * \param reinitialize_device If \p true, i.e. by default, reinitialize the device.
   * \sa initializeServicesAndWait(), waitForInitialization(), waitForServices()
   */
  void resetServicesAndWait(const bool &reinitialize_device = true);

  /**
   * Wait until the device is initialized.
   * \sa initializeDevice()
   */
  void waitForInitialization();
  
    /**
   * Wait until all the services advertised by the Communication Handler are active, then reinitialize the device to
   * avoid disconnection problems.
   * \sa initializeServicesAndWait(), resetServicesAndWait()
   */
  void waitForServices();
};
typedef std::shared_ptr<IMUHW> IMUHWPtr;
}  // namespace imu_hardware_interface

#endif // IMU_HARDWARE_INTERFACE_H