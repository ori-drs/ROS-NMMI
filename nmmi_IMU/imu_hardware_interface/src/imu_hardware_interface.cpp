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

#include <imu_hardware_interface/imu_hardware_interface.h>

using namespace imu_hardware_interface;

IMUHW::IMUHW() 
  : qbDeviceHW(std::make_shared<imu_transmission_interface::IMUVirtualTransmission>(), {"fake_joint"}, {"fake_joint"}) {
}

IMUHW::~IMUHW() {

}

void IMUHW::compute_quat_ext(int n, const ros::Duration& period, int read_fail) {

  float q1, q2 ,q3 ,q4;
  float sx, sy, sz;
  Eigen::Vector3d aP,fa;
  Eigen::Vector4d gP,qL,qdot,Napla;
  Eigen::MatrixXd Ja(3,4); 

  float Min_Acc_Norm = 0.8;
  float Max_Acc_Norm = 1.2;
  float beta = 2.0;
  //float sampleFreq = 50;
  float Gyro_Th = 18;

  // Check and Prepare Accelerations
  aP(0)  = Acc_.at(3*n);  
  aP(1)  = Acc_.at(3*n+1);  
  aP(2)  = Acc_.at(3*n+2);

  if (aP.norm() < Min_Acc_Norm) {
    aP(0) = Acc_old_.at(3*n); 
    aP(1) = Acc_old_.at(3*n+1); 
    aP(2) = Acc_old_.at(3*n+2); 
  }
  if (aP.norm() > Max_Acc_Norm) {
    aP(0) = Acc_old_.at(3*n); 
    aP(1) = Acc_old_.at(3*n+1); 
    aP(2) = Acc_old_.at(3*n+2); 
  } 

  aP = aP / aP.norm();
  sx = aP(0); 
  sy = aP(1); 
  sz = aP(2); 
  
  // Check And Prepare Gyros
  std::vector<double> Gyro_q(Gyro_);
  if (fabs(Gyro_.at(3*n)) < Gyro_Th)   {Gyro_q.at(3*n+0) = 0;}
  if (fabs(Gyro_.at(3*n+1)) < Gyro_Th) {Gyro_q.at(3*n+1) = 0;}
  if (fabs(Gyro_.at(3*n+2)) < Gyro_Th) {Gyro_q.at(3*n+2) = 0;}
  gP(0)  = 0; 
  gP(1)  = Gyro_q.at(3*n);  
  gP(2)  = Gyro_q.at(3*n+1);  
  gP(3)  = Gyro_q.at(3*n+2);
  gP = gP*(M_PI/180);
  
  if (!read_fail) {
    // If sensors reading is good, update quaternion; otherwise, leave it as is

    // Prepare Quaternion
    q1 = Ext_Quat_.at(4*n + 0);  
    q2 = Ext_Quat_.at(4*n + 1); 
    q3 = Ext_Quat_.at(4*n + 2); 
    q4 = Ext_Quat_.at(4*n + 3);

    qL(0) = q1;
    qL(1) = q2;
    qL(2) = q3;
    qL(3) = q4;

    // Cost Function
    fa(0) =  2*(q2*q4-q1*q3) - sx; 
    fa(1) =  2*(q1*q2 + q3*q4) - sy;
    fa(2) =  2*(0.5 - q2*q2 -q3*q3) - sz;   
    
    // Compute the Jacobian
    Ja << -2*q3,  2*q4, -2*q1,  2*q2,
           2*q2,  2*q1,  2*q4,  2*q3,
              0, -4*q2, -4*q3,     0;
    
    // Compute the Napla
    Napla = Ja.transpose() * fa;
    
    //qdot = 0.5*QxQ( qL,g ) - ( beta*Napla );        
    qdot(0) = qL(0)*gP(0) - (qL(1)*gP(1) + qL(2)*gP(2) + qL(3)*gP(3));
    qdot(1) = qL(0)*gP(1) + qL(1)*gP(0) + (qL(2)*gP(3) - qL(3)*gP(2));
    qdot(2) = qL(0)*gP(2) + qL(2)*gP(0) + (qL(3)*gP(1) - qL(1)*gP(3));
    qdot(3) = qL(0)*gP(3) + qL(3)*gP(0) + (qL(1)*gP(2) - qL(2)*gP(1));

    qdot = 0.5*qdot - (beta*Napla);

    qL = qL + qdot * period.toSec();  //qL = qL + qdot / sampleFreq;
    qL = qL /qL.norm();

    Ext_Quat_.at(4*n + 0) = qL(0);
    Ext_Quat_.at(4*n + 1) = qL(1);
    Ext_Quat_.at(4*n + 2) = qL(2);
    Ext_Quat_.at(4*n + 3) = qL(3);
  }

  Acc_old_ = Acc_;
}

int IMUHW::getIMUvalues() {
  if (services_.at("get_imu_values")) {
    nmmi_srvs::GetIMUValues srv;
    srv.request.id = device_.id;
    srv.request.max_repeats = device_.max_repeats;
    srv.request.get_values = get_imu_values_;
    srv.request.imu_table = imu_table_msg_;
    srv.request.custom_read_timeout = custom_read_timeout_;
    services_.at("get_imu_values").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[IMU DeviceHW] cannot get IMU values from device [" << device_.id << "].");
      return srv.response.failures;  // positions and currents are not updated
    }

    Acc_.resize(srv.response.acc.size());
    Acc_old_.resize(srv.response.acc.size());
    for (int i=0; i<srv.response.acc.size(); i++) {
      Acc_.at(i) = srv.response.acc.at(i);
    }

    Gyro_.resize(srv.response.gyro.size());
    for (int i=0; i<srv.response.gyro.size(); i++) {
      Gyro_.at(i) = srv.response.gyro.at(i);
    }

    Mag_.resize(srv.response.mag.size());
    for (int i=0; i<srv.response.mag.size(); i++) {
      Mag_.at(i) = srv.response.mag.at(i);
    }

    Quat_.resize(srv.response.quat.size());
    Ext_Quat_.resize(srv.response.quat.size());
    for (int i=0; i<srv.response.quat.size(); i++) {
      Quat_.at(i) = srv.response.quat.at(i);
    }

    Temp_.resize(srv.response.temp.size());
    for (int i=0; i<srv.response.temp.size(); i++) {
      Temp_.at(i) = srv.response.temp.at(i);
    }

    time_stamp_ = srv.response.stamp;
    return srv.response.failures;
  }
  ROS_WARN_STREAM_NAMED("device_hw", "[IMU DeviceHW] service [get_IMU] seems no longer advertised. Trying to reconnect...");
  resetServicesAndWait();
  return -1;
}

std::vector<std::string> IMUHW::getJoints() {
  return joints_.names;
}

bool IMUHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) {

  node_handle_ = robot_hw_nh;
  if (!robot_hw_nh.getParam("device_name", device_.name)) {
    ROS_ERROR_STREAM_NAMED("device_hw", "[IMU DeviceHW] cannot retrieve 'device_name' from the Parameter Service [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }
  if (!robot_hw_nh.getParam("device_id", device_.id)) {
    ROS_ERROR_STREAM_NAMED("device_hw", "[IMU DeviceHW] cannot retrieve 'device_id' from the Parameter Service [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }

  imuboard_pub_state_ = robot_hw_nh.advertise<nmmi_msgs::State>("state", 1);
  imuboard_pub_acc_   = robot_hw_nh.advertise<nmmi_msgs::inertialSensorArray>("acc", 1);
  imuboard_pub_gyro_  = robot_hw_nh.advertise<nmmi_msgs::inertialSensorArray>("gyro", 1);
  imuboard_pub_mag_   = robot_hw_nh.advertise<nmmi_msgs::inertialSensorArray>("mag", 1);
  imuboard_pub_quat_  = robot_hw_nh.advertise<nmmi_msgs::quaternionArray>("quat", 1);
  imuboard_pub_temp_  = robot_hw_nh.advertise<nmmi_msgs::temperatureArray>("temp", 1);
  imuboard_pub_angles_= robot_hw_nh.advertise<nmmi_msgs::anglesArray>("angles", 1);

  interfaces_.initialize(this, joints_);
  transmission_.initialize(robot_hw_nh.param<std::string>("transmission", "transmission"), actuators_, joints_);

  // initialize IMU services before initializing the device
  initializeIMUServicesAndWait();
  waitForInitialization();
  ROS_INFO_STREAM(getInfo());

  // if the device interface initialization has succeed the device info have been retrieved
  std::static_pointer_cast<imu_transmission_interface::IMUVirtualTransmission>(transmission_.getTransmission());

  return true;
}

int IMUHW::initializeDevice() {

  uint8_t PARAM_SLOT_BYTES = 50;
  uint8_t num_imus_id_params = 7;
  uint8_t num_mag_cal_params = 0;
  uint8_t first_imu_parameter = 2;
  int v = 0;
  int num_of_params;

  if (services_.at("initialize_nmmi_device")) {
    qb_device_srvs::InitializeDevice srv;
    srv.request.id = device_.id;
    srv.request.activate = node_handle_.param<bool>("activate_on_initialization", true);
    srv.request.rescan = node_handle_.param<bool>("rescan_on_initialization", false);
    int max_repeats = node_handle_.param<int>("max_repeats", 3);
    srv.request.max_repeats = max_repeats;
    services_.at("initialize_nmmi_device").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[IMU DeviceHW] cannot initialize device [" << device_.id << "].");
      return -1;
    }
    device_.max_repeats = max_repeats;
    device_.serial_port = srv.response.info.serial_port;

    device_info_.id = device_.id;
    device_info_.serial_port = device_.serial_port;
    device_info_.max_repeats = device_.max_repeats;
    device_info_.get_currents = device_.get_currents;
    device_info_.get_positions = device_.get_positions;
    device_info_.get_distinct_packages = device_.get_distinct_packages;
    device_info_.set_commands = device_.set_commands;
    device_info_.set_commands_async = device_.set_commands_async;
    device_info_.position_limits = device_.position_limits;
    device_info_.encoder_resolutions = device_.encoder_resolutions;

    get_imu_values_ = node_handle_.param<bool>("get_imu_values", false);
    compute_quaternions_node_ = node_handle_.param<bool>("compute_quaternions_node", false);
    compute_angles_ = node_handle_.param<bool>("compute_angles", false);

    ROS_INFO_STREAM_NAMED("device_hw", "[IMU DeviceHW] device [" << device_.id << "] is initialized.");
  }

  if (services_.at("get_imu_param")) {
    nmmi_srvs::GetIMUParam param_srv;
    param_srv.request.id = device_.id;
    services_.at("get_imu_param").call(param_srv);
    if (!param_srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[IMU DeviceHW] cannot read parameters from device [" << device_.id << "].");
      return -1;
    }

    if (param_srv.response.old_board == true){
      // If the response variable 'old_board' is set to true, the connected board is a PSoC3 board instead of a STM32 or PSoC5 board
      // so update the number of id_params to the right value
      num_imus_id_params = 6;
    } 
    else {
      num_imus_id_params = 7;
    }

    custom_read_timeout_ = !param_srv.response.old_board;  // If the connected board is a STM32 or PSoC5 board, set custom read timeout as default
  
    num_of_params = (uint8_t)param_srv.response.message[5];

    //param_srv.response.message[6] <-> packet_data[2] on the firmware
    n_imu_ = (uint8_t)param_srv.response.message[8];
    //printf("Number of connected IMUs: %d\n", n_imu_);
    if (n_imu_ <= 0) {
      ROS_INFO_STREAM_NAMED("device_hw", "[IMU DeviceHW] device [" << device_.id << "] has no IMU connected");
      return -1;
    }
  
    // Compute number of read parameters depending on global_args.n_imu and
    // update packet_length
    num_mag_cal_params = (n_imu_ / 2);
    if ( (n_imu_ - num_mag_cal_params*2) > 0 ) num_mag_cal_params++;

    ROS_INFO_STREAM_NAMED("device_hw", "[IMU DeviceHW] device [" << device_.id << "] parameters table:");

    //ids_ = (uint8_t *) calloc(n_imu_, sizeof(uint8_t));
    ids_.resize(n_imu_);
    v = 0;

    for (int k = 1; k <= num_imus_id_params; k++){
      if (param_srv.response.message[k*PARAM_SLOT_BYTES + 8] != -1) {
        ids_[v] = param_srv.response.message[k*PARAM_SLOT_BYTES + 8];
        v++;
      }
      if (param_srv.response.message[k*PARAM_SLOT_BYTES + 9] != -1) {
        ids_[v] = param_srv.response.message[k*PARAM_SLOT_BYTES + 9];
        v++;
      }
      if (param_srv.response.message[k*PARAM_SLOT_BYTES + 10] != -1) {
        ids_[v] = param_srv.response.message[k*PARAM_SLOT_BYTES + 10];
        v++;
      }
    }

    // Retrieve magnetometer calibration parameters
    //mag_cal_ = (uint8_t *) calloc(n_imu_, 3*sizeof(uint8_t));
    imu_table_msg_.mag_cal.resize(n_imu_*3*sizeof(uint8_t));
    v = 0;
    for (int k=1; k <= num_mag_cal_params; k++) {
      imu_table_msg_.mag_cal[3*v + 0] = param_srv.response.message[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 8];
      imu_table_msg_.mag_cal[3*v + 1] = param_srv.response.message[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 9];
      imu_table_msg_.mag_cal[3*v + 2] = param_srv.response.message[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 10];
      ROS_INFO_STREAM_NAMED("device_hw", "[IMU DeviceHW] MAG PARAM: " << (int)imu_table_msg_.mag_cal[3*v + 0] << " " << (int)imu_table_msg_.mag_cal[3*v + 1] << " " << (int)imu_table_msg_.mag_cal[3*v + 2]);
      v++;
      
      if (param_srv.response.message[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 7] == 6) {
        imu_table_msg_.mag_cal[3*v + 0] = param_srv.response.message[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 11];
        imu_table_msg_.mag_cal[3*v + 1] = param_srv.response.message[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 12];
        imu_table_msg_.mag_cal[3*v + 2] = param_srv.response.message[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 13];
        ROS_INFO_STREAM_NAMED("device_hw", "[IMU DeviceHW] MAG PARAM: " << (int)imu_table_msg_.mag_cal[3*v + 0] << " " << (int)imu_table_msg_.mag_cal[3*v + 1] << " " << (int)imu_table_msg_.mag_cal[3*v + 2]);
        v++;
      }
    }
  
    first_imu_parameter = 1 + num_imus_id_params + num_mag_cal_params + 1;
    //imu_table_msg_.imu_table = (uint8_t *) calloc(n_imu_, 5*sizeof(uint8_t));
    imu_table_msg_.imu_table.resize(n_imu_*5*sizeof(uint8_t));
    for (int i=0; i< n_imu_; i++){
      imu_table_msg_.imu_table[5*i + 0] = param_srv.response.message[first_imu_parameter*PARAM_SLOT_BYTES + 8 + 50*i];
      imu_table_msg_.imu_table[5*i + 1] = param_srv.response.message[first_imu_parameter*PARAM_SLOT_BYTES+ 9 + 50*i];
      imu_table_msg_.imu_table[5*i + 2] = param_srv.response.message[first_imu_parameter*PARAM_SLOT_BYTES + 10 + 50*i];
      imu_table_msg_.imu_table[5*i + 3] = param_srv.response.message[first_imu_parameter*PARAM_SLOT_BYTES + 11 + 50*i];
      imu_table_msg_.imu_table[5*i + 4] = param_srv.response.message[first_imu_parameter*PARAM_SLOT_BYTES + 12 + 50*i];
      ROS_INFO_STREAM_NAMED("device_hw", "[IMU DeviceHW] ID: " << (int)ids_[i] << " - " << (int)imu_table_msg_.imu_table[5*i + 0] << ", " << (int)imu_table_msg_.imu_table[5*i + 1] << ", " << (int)imu_table_msg_.imu_table[5*i + 2] << ", " << (int)imu_table_msg_.imu_table[5*i + 3] << ", " << (int)imu_table_msg_.imu_table[5*i + 4]);     
    }
  
    // Imu values is a 3 sensors x 3 axes x n_imu_ values
    imu_values_ = (float *) calloc(n_imu_, 3*3*sizeof(float)+4*sizeof(float)+sizeof(float));
 
    // Initialize imu_table msg
    imu_table_msg_.id = device_.id;
    imu_table_msg_.n_imu = n_imu_;

    // std::vector default initialization to zero value
    Acc_.resize(n_imu_*3);
    Gyro_.resize(n_imu_*3);
    Mag_.resize(n_imu_*3);
    Quat_.resize(n_imu_*4);
    Temp_.resize(n_imu_);
    Acc_old_.resize(n_imu_*3);
    Ext_Quat_.resize(n_imu_*4);

    for (int i=0; i<n_imu_; i++) {
      Ext_Quat_.at(4*i+0) = 1.0;
    }

    // If both services were good, return 0
    return 0;
  }

  ROS_WARN_STREAM_NAMED("device_hw", "[IMU DeviceHW] service [initialize_device] is no longer advertised.");
  resetServicesAndWait(false);
  return -1;
}

void IMUHW::initializeIMUServicesAndWait() {
  services_["get_imu_values"] = node_handle_.serviceClient<nmmi_srvs::GetIMUValues>("/communication_handler/get_imu_values", true);
  services_["get_imu_param"] = node_handle_.serviceClient<nmmi_srvs::GetIMUParam>("/communication_handler/get_imu_param", true);
  services_["initialize_nmmi_device"] = node_handle_.serviceClient<qb_device_srvs::InitializeDevice>("/communication_handler/initialize_nmmi_device", true);
  waitForServices();
}

void IMUHW::publish() {
  nmmi_msgs::State msg;
  nmmi_msgs::inertialSensor tmp_acc, tmp_gyro, tmp_mag;
  nmmi_msgs::inertialSensorArray acc, gyro, mag;
  nmmi_msgs::quaternion tmp_quat;
  nmmi_msgs::quaternionArray quat;
  nmmi_msgs::temperature tmp_temp;
  nmmi_msgs::temperatureArray temp;
  nmmi_msgs::angles tmp_angles;
  nmmi_msgs::anglesArray angles;

  msg.id = device_.id;
  msg.is_reliable = consecutive_failures_ >= 0 && consecutive_failures_ <= device_.max_repeats;
  msg.consecutive_failures = consecutive_failures_;
  msg.stamp = time_stamp_;

  for (int i=0; i<n_imu_; i++) {
    nmmi_msgs::ResourceData msg_rd;
    msg_rd.acc.resize(3);
    msg_rd.gyro.resize(3);
    msg_rd.mag.resize(3);
    msg_rd.quat.resize(4);
    msg_rd.temp.resize(1);

    msg_rd.imu_id = ids_.at(i);
    for (int j=0; Acc_.size() && j<3; j++) {
      msg_rd.acc.at(j) = Acc_.at(3*i + j);
    }
    for (int j=0; Gyro_.size() && j<3; j++) {
      msg_rd.gyro.at(j) = Gyro_.at(3*i + j);
    }
    for (int j=0; Mag_.size() && j<3; j++) {
      msg_rd.mag.at(j) = Mag_.at(3*i + j);
    }
    for (int j=0; Quat_.size() && j<4; j++) {
      if (imu_table_msg_.imu_table.at(5*i + 3)){
        msg_rd.quat.at(j) = Quat_.at(4*i + j);
      } 
      else {
        msg_rd.quat.at(j) = Ext_Quat_.at(4*i + j);
      }
    }
    if (Temp_.size()) {
      msg_rd.temp.at(0) = Temp_.at(i);
    }

    //msg_rd.acc
    msg.s.push_back(msg_rd);

    // Fill single topics
    if (imu_table_msg_.imu_table.at(5*i + 0)) {
      tmp_acc.board_id = device_.id;
      tmp_acc.id = ids_.at(i);
      tmp_acc.x = msg_rd.acc.at(0);
      tmp_acc.y = msg_rd.acc.at(1);
      tmp_acc.z = msg_rd.acc.at(2);
      acc.m.push_back(tmp_acc);
    }
    if (imu_table_msg_.imu_table.at(5*i + 1)) {
      tmp_gyro.board_id = device_.id;
      tmp_gyro.id = ids_.at(i);
      tmp_gyro.x = msg_rd.gyro.at(0);
      tmp_gyro.y = msg_rd.gyro.at(1);
      tmp_gyro.z = msg_rd.gyro.at(2);
      gyro.m.push_back(tmp_gyro);
    }
    if (imu_table_msg_.imu_table.at(5*i + 2)) {
      tmp_mag.board_id = device_.id;
      tmp_mag.id = ids_.at(i);
      tmp_mag.x = msg_rd.mag.at(0);
      tmp_mag.y = msg_rd.mag.at(1);
      tmp_mag.z = msg_rd.mag.at(2);
      mag.m.push_back(tmp_mag);
    }
    if (imu_table_msg_.imu_table.at(5*i + 3) || (imu_table_msg_.imu_table.at(5*i + 0) && imu_table_msg_.imu_table.at(5*i + 1))) {
      tmp_quat.board_id = device_.id;
      tmp_quat.id = ids_.at(i);
      tmp_quat.w = msg_rd.quat.at(0);
      tmp_quat.x = msg_rd.quat.at(1);
      tmp_quat.y = msg_rd.quat.at(2);
      tmp_quat.z = msg_rd.quat.at(3);
      quat.m.push_back(tmp_quat);
    } 
    if (imu_table_msg_.imu_table.at(5*i + 4)) {
      tmp_temp.board_id = device_.id;
      tmp_temp.id = ids_.at(i);
      tmp_temp.value = msg_rd.temp.at(0);
      temp.m.push_back(tmp_temp);
    }

    if (compute_angles_ && (imu_table_msg_.imu_table.at(5*i + 3) || 
      (imu_table_msg_.imu_table.at(5*i + 0) && imu_table_msg_.imu_table.at(5*i + 1)))){
      std::vector<float> q(msg_rd.quat);
      std::vector<float> a(3);
      quat_to_angles(q, a);
      tmp_angles.board_id = device_.id;
      tmp_angles.id = ids_.at(i);
      tmp_angles.r = a.at(0);
      tmp_angles.p = a.at(1);
      tmp_angles.y = a.at(2);
      angles.m.push_back(tmp_angles);
    }
}

if (get_imu_values_){
  imuboard_pub_acc_.publish(acc);
  imuboard_pub_gyro_.publish(gyro);      
  imuboard_pub_mag_.publish(mag); 
  imuboard_pub_quat_.publish(quat);
  imuboard_pub_temp_.publish(temp);
}
if (compute_angles_){
  imuboard_pub_angles_.publish(angles);
}
  imuboard_pub_state_.publish(msg);
}

void IMUHW::quat_to_angles(std::vector<float> q, std::vector<float>&a){
  // q = [w, x, y, z]
  double w = q.at(0);
  double x = q.at(1);
  double y = q.at(2);
  double z = q.at(3);

  a.at(0) = -atan2(2*x*w - 2*y*z, 1-2*x*x - 2*z*z)*(180/M_PI); //ROLL
  a.at(1) = -atan2(2*y*w - 2*x*z, 1-2*y*y - 2*z*z)*(180/M_PI); // PITCH 
  a.at(2) = asin(2*x*y + 2*z*w)*(180/M_PI); //YAW  
}

void IMUHW::read(const ros::Time& time, const ros::Duration& period) {

  // Call associated service
  consecutive_failures_ = getIMUvalues();

  if (get_imu_values_ && compute_quaternions_node_){
    // Update Ext_Quat_ if needed
    for (int i=0; i< n_imu_; i++) {
      if (!imu_table_msg_.imu_table.at(5*i + 3) && imu_table_msg_.imu_table.at(5*i + 0) && imu_table_msg_.imu_table.at(5*i + 1)){  // acc and gyro needed
        compute_quat_ext(i, period, consecutive_failures_);
      }
    }
  }

  // propagate current actuator state to joints
  transmission_.actuator_to_joint_state.propagate();

  // make data available for other ROS nodes
  publish();
}

void IMUHW::resetServicesAndWait(const bool &reinitialize_device) {
  waitForServices();
  // reset all the service clients in case they were not yet advertised during initialization
  initializeIMUServicesAndWait();
  if (reinitialize_device) {
    waitForInitialization();
  }
}

void IMUHW::waitForInitialization() {
  while(initializeDevice()) {
    ros::Duration(1.0).sleep();
  }
}

void IMUHW::waitForServices() {
  for (auto &service : services_) {
    service.second.waitForExistence();
  }
  ROS_INFO_STREAM_NAMED("device_hw", "[IMU DeviceHW] is connected to all the services advertise by [CommunicationHandler].");
}

void IMUHW::write(const ros::Time& time, const ros::Duration& period) {
  // Empty function since IMU has no actuators, so it does not need to send actuator command to the hardware
}

PLUGINLIB_EXPORT_CLASS(imu_hardware_interface::IMUHW, hardware_interface::RobotHW)
