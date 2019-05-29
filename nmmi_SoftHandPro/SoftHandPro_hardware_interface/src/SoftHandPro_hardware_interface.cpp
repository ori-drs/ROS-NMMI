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

#include <SoftHandPro_hardware_interface/SoftHandPro_hardware_interface.h>

using namespace SoftHandPro_hardware_interface;

SoftHandProHW::SoftHandProHW()
    : qbDeviceHW(std::make_shared<SoftHandPro_transmission_interface::SoftHandProVirtualTransmission>(), {"synergy_joint"}, {"synergy_joint"}) {
}

SoftHandProHW::~SoftHandProHW() {

}

std::vector<std::string> SoftHandProHW::getJoints() {
  return joints_.names;
}

bool SoftHandProHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) {
  
  if (!qb_device_hardware_interface::qbDeviceHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // if the device interface initialization has succeed the device info have been retrieved
  std::static_pointer_cast<SoftHandPro_transmission_interface::SoftHandProVirtualTransmission>(transmission_.getTransmission())->setPositionFactor(device_.position_limits.at(1));
  return true;
}

void SoftHandProHW::read(const ros::Time& time, const ros::Duration& period) {
  // read actuator state from the hardware (convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::read(time, period);
}

void SoftHandProHW::write(const ros::Time& time, const ros::Duration& period) {
  // send actuator command to the hardware (saturate and convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::write(time, period);
}

PLUGINLIB_EXPORT_CLASS(SoftHandPro_hardware_interface::SoftHandProHW, hardware_interface::RobotHW)