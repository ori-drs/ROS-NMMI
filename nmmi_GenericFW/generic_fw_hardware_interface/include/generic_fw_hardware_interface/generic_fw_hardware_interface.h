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

#ifndef GENERIC_FW_HARDWARE_INTERFACE_H
#define GENERIC_FW_HARDWARE_INTERFACE_H

// ROS libraries
#include <pluginlib/class_list_macros.hpp>

// internal libraries
#include <qb_device_hardware_interface/qb_device_hardware_interface.h>
#include <generic_fw_hardware_interface/generic_fw_transmission_interface.h>
#include <nmmi_msgs/nmmi_msgs.h>
#include <nmmi_srvs/nmmi_srvs.h>

namespace generic_fw_hardware_interface {
/**
 * The \em Generic FW HardWare interface implements the specific structures to manage the communication with a
 * \em NMMI device with generic fw. It exploits the features provided by the base device-independent hardware interface and the
 * specific transmission interface.
 * \sa qb_device_hardware_interface::qbDeviceHW, generic_fw_transmission_interface::GenericFWVirtualTransmission
 */
class GenericFWHW : public qb_device_hardware_interface::qbDeviceHW {
 public:
  /**
   * Initialize the \p qb_device_hardware_interface::qbDeviceHW with the specific transmission interface and actuator
   * and joint names.
   * \sa generic_fw_transmission_interface::GenericFWVirtualTransmission
   */
  GenericFWHW();

  /**
   * Do nothing.
   */
  virtual ~GenericFWHW();
  
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

  int*  adc_values_;
  int*  encoder_values_;
  bool  get_adc_values_;
  bool  get_encoders_values_;
  bool  is_reliable_;
  int   adc_consecutive_failures_;
  int   encoder_consecutive_failures_;

  std::vector<int16_t> Adc_Raw_;
  std::vector<uint8_t> Adc_Map_;
  uint8_t used_adc_channels_;
  ros::Time adc_time_stamp_;
  std::vector<uint16_t> Encoder_Raw_;
  std::vector<uint8_t> Encoder_Map_;
  uint8_t num_encoder_conf_total_;
  ros::Time enc_time_stamp_;

  // Publisher variables
  ros::Publisher generic_pub_adc_;
  ros::Publisher generic_pub_adc_state_;
  ros::Publisher generic_pub_encoders_;
  ros::Publisher generic_pub_encoders_state_;
      
  int getADCRawvalues();

  int getEncoderRawvalues();
  
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
  void initializeGenericFWServicesAndWait();
  
  /**
   * Construct a \p qb_device_msgs::StateStamped message of the whole device state with the data retrieved during the
   * \p read() and publish it to a namespaced \p "~state" topic.
   * \sa read()
   */
  void publish();


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
typedef std::shared_ptr<GenericFWHW> GenericFWHWPtr;
}  // namespace generic_fw_hardware_interface

#endif // GENERIC_FW_HARDWARE_INTERFACE_H