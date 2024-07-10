#include "fcat/fcat_utils.hpp"
#include "fcat/fcat_services.hpp"
#include "jsd/jsd_print.h"
#include "fastcat/jsd/actuator.h"

#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

//Fcatservices::~Fcatservices(){ }

FcatServices::FcatServices() 
    : Node("fcat_services", "fcat_services"),
      module_state_last_recv_time_(0),
      act_states_last_recv_time_(0),
      pid_states_last_recv_time_(0),
      srv_state_(FCAT_SRV_STATE_IDLE_CHECKING) 
{
  cb_group_blocking_     = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_non_blocking_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  //this->declare_parameter<double>("loop_rate_hz", 100);
  //this->get_parameter("loop_rate_hz", loop_rate_hz_);

  //this->declare_parameter<double>("position_tolerance", 1.0e-2);
  //this->get_parameter("position_tolerance", position_tolerance_);

  //this->declare_parameter<double>("velocity_tolerance", 1.0e-2);
  //this->get_parameter("velocity_tolerance", velocity_tolerance_);

  //this->declare_parameter<double>("current_tolerance", 1.0e-2);
  //this->get_parameter("current_tolerance", current_tolerance_);

  // Init Parameters
  pub_sub_ns_ =
      this->declare_parameter<std::string>("pub_sub_namespace", "/fcat/");

  sdo_app_id_ = static_cast<uint16_t>(
      this->declare_parameter<std::uint16_t>("starting_sdo_app_id", 1000,
                              0, 65535));

  max_sdo_queue_size_ = static_cast<size_t>(t
      his->declare_parameter<std::size_t>("max_sdo_queue_size", 32,
      1, 10000));

  // Runtime Parameters
  this->declare_parameter<std::uint16_t>(
      "idle_persist_rti", 5,
      1, 1000);

  this->declare_parameter<std::double>("tolerance", 1.0e-8,
                                0, 10);

  this->declare_parameter<std::double>(
      "sdo_wait_duration_sec", 2.0,
      0, 60);

  InitSubscribers();
  InitPublishers();
  InitServices();

  //module_state_last_recv_time_ = 0;
  //act_states_last_recv_time_ = 0;
  //pid_states_last_recv_time_ = 0;
  //MSG("Spinning...");

  // Call this only after setting all initialization array params
  PreventArrayParamSet();
  InitializeTimerRate();

  //   callbackgroups, and rclcpp::Rate instead
  rate_ = std::make_unique<rclcpp::Rate>(this->GetTimerRate());

  MSG("Spinning...");
}

void FcatServices::InitSubscribers()
{
  // auto sub_opts = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
  // sub_opts.callback_group = cb_group_non_blocking_;

  // fcat_module_state_sub_ = this->create_subscription<fcat_msgs::msg::ModuleState>(
  //     "state/module_state",
  //     FCAT_services_QOS_SUBS, 
  //     std::bind(&FcatServices::FcatModuleStateCb, this, _1),
  //     sub_opts);

  // act_states_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorStates>(
  //     "state/actuators", 
  //     FCAT_services_QOS_SUBS, 
  //     std::bind(&FcatServices::ActuatorStatesCb, this, _1),
  //     sub_opts);

  // pid_states_sub_ = this->create_subscription<fcat_msgs::msg::PidStates>(
  //     "state/pids", 
  //     FCAT_services_QOS_SUBS, 
  //     std::bind(&FcatServices::PidStatesCb, this, _1),
  //     sub_opts);
  auto sub_opts =
      rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
  sub_opts.callback_group = cb_group_non_blocking_;

  subscriptions_.push_back(
      this->create_subscription<fcat_msgs::msg::ModuleState>(
          pub_sub_ns_ + "state/module_state", subscription_qos_,
          std::bind(&FcatServices::FcatModuleStateCb, this, _1), sub_opts));

  subscriptions_.push_back(
      this->create_subscription<fcat_msgs::msg::ActuatorStates>(
          pub_sub_ns_ + "state/actuators", subscription_qos_,
          std::bind(&FcatServices::ActuatorStatesCb, this, _1), sub_opts));

  subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::PidStates>(
      pub_sub_ns_ + "state/pids", subscription_qos_,
      std::bind(&FcatServices::PidStatesCb, this, _1), sub_opts));

  subscriptions_.push_back(
      this->create_subscription<fcat_msgs::msg::AsyncSdoResponse>(
          pub_sub_ns_ + "state/async_sdo_response", subscription_qos_,
          std::bind(&FcatServices::AsyncSdoResponseCb, this, _1), sub_opts));
}

void FcatServices::InitPublishers()
{
  // act_prof_pos_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorProfPosCmd>(
  //     "cmd/actuator_prof_pos", FCAT_services_PUB_QUEUE_SIZE);

  // act_prof_vel_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorProfVelCmd>(
  //     "cmd/actuator_prof_vel", FCAT_services_PUB_QUEUE_SIZE);

  // act_prof_torque_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorProfTorqueCmd>(
  //     "cmd/actuator_prof_torque", FCAT_services_PUB_QUEUE_SIZE);

  // act_calibrate_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorCalibrateCmd>(
  //     "cmd/actuator_calibrate", FCAT_services_PUB_QUEUE_SIZE);

  // pid_activate_pub_ = this->create_publisher<fcat_msgs::msg::PidActivateCmd>(
  //     "cmd/pid_activate", FCAT_services_PUB_QUEUE_SIZE);
  act_prof_pos_pub_ =
      this->create_publisher<fcat_msgs::msg::ActuatorProfPosCmd>(
          pub_sub_ns_ + "impl/actuator_prof_pos", publisher_queue_size_);

  act_prof_vel_pub_ =
      this->create_publisher<fcat_msgs::msg::ActuatorProfVelCmd>(
          pub_sub_ns_ + "impl/actuator_prof_vel", publisher_queue_size_);

  act_prof_torque_pub_ =
      this->create_publisher<fcat_msgs::msg::ActuatorProfTorqueCmd>(
          pub_sub_ns_ + "impl/actuator_prof_torque", publisher_queue_size_);

  act_digital_output_pub_ =
      this->create_publisher<fcat_msgs::msg::ActuatorSetDigitalOutputCmd>(
          pub_sub_ns_ + "impl/actuator_set_digital_output",
          publisher_queue_size_);

  act_calibrate_pub_ =
      this->create_publisher<fcat_msgs::msg::ActuatorCalibrateCmd>(
          pub_sub_ns_ + "impl/actuator_calibrate", publisher_queue_size_);

  pid_activate_pub_ = this->create_publisher<fcat_msgs::msg::PidActivateCmd>(
      pub_sub_ns_ + "impl/pid_activate", publisher_queue_size_);

  async_sdo_write_pub_ =
      this->create_publisher<fcat_msgs::msg::AsyncSdoWriteCmd>(
          pub_sub_ns_ + "impl/async_sdo_write", publisher_queue_size_);

  async_sdo_read_pub_ = this->create_publisher<fcat_msgs::msg::AsyncSdoReadCmd>(
      pub_sub_ns_ + "impl/async_sdo_read", publisher_queue_size_);
}

void FcatServices::InitServices()
{
  // act_prof_pos_srv_=
  //   this->create_service<fcat_msgs::srv::ActuatorProfPosCmd>(
  //       "service/actuator_prof_pos", 
  //       std::bind(&FcatServices::ActuatorProfPosSrvCb, this, _1, _2),
  //       FCAT_services_QOS_services,
  //       cb_group_blocking_);

  // act_prof_vel_srv_=
  //   this->create_service<fcat_msgs::srv::ActuatorProfVelCmd>(
  //       "service/actuator_prof_vel", 
  //       std::bind(&FcatServices::ActuatorProfVelSrvCb, this, _1, _2),
  //       FCAT_services_QOS_services,
  //       cb_group_blocking_);

  // act_prof_torque_srv_=
  //   this->create_service<fcat_msgs::srv::ActuatorProfTorqueCmd>(
  //       "service/actuator_prof_torque", 
  //       std::bind(&FcatServices::ActuatorProfTorqueSrvCb, this, _1, _2),
  //       FCAT_services_QOS_services,
  //       cb_group_blocking_);

  // act_calibrate_srv_=
  //   this->create_service<fcat_msgs::srv::ActuatorCalibrateCmd>(
  //       "service/actuator_calibrate", 
  //       std::bind(&FcatServices::ActuatorCalibrateSrvCb, this, _1, _2),
  //       FCAT_services_QOS_services,
  //       cb_group_blocking_);

  // pid_activate_srv_=
  //   this->create_service<fcat_msgs::srv::PidActivateCmd>(
  //       "service/pid_activate", 
  //       std::bind(&FcatServices::PidActivateSrvCb, this, _1, _2),
  //       FCAT_services_QOS_services,
  //       cb_group_blocking_);
  DeclareServiceCommand<fcat_msgs::srv::ActuatorProfPosService>(
      this, pub_sub_ns_ + "srv/actuator_prof_pos",
      &FcatServices::ActuatorProfPosSrvCb, services_qos_, cb_group_blocking_,
      CommandDescriptor(
          "Command a Profiled Position motion profile to the Actuator",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "target_position",
               /*description*/ "Goal Position target of the motion profile",
               /*Units*/ "EU"),
           CommandArgumentDescriptor(
               /*arg name*/ "profile_velocity",
               /*description*/ "The desired cruise rate of the motion profile",
               /*Units*/ "EU / sec"),
           CommandArgumentDescriptor(
               /*arg name*/ "profile_accel",
               /*description*/ "The desired acceleration of the motion profile",
               /*Units*/ "EU/sec^2"),
           CommandArgumentDescriptor(
               /*arg name*/ "relative",
               /*description*/ "If true, the target_position will computed "
                               "relative "
                               "to actual position")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorProfVelService>(
      this, pub_sub_ns_ + "srv/actuator_prof_vel",
      &FcatServices::ActuatorProfVelSrvCb, services_qos_, cb_group_blocking_,
      CommandDescriptor(
          "Command a Profiled Velocity motion profile to the Actuator",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "target_velocity",
               /*description*/ "The desired cruise rate of the motion profile",
               /*Units*/ "EU/sec"),
           CommandArgumentDescriptor(
               /*arg name*/ "profile_accel",
               /*description*/ "The desired acceleration of the motion profile",
               /*Units*/ "EU/sec^2"),
           CommandArgumentDescriptor(
               /*arg name*/ "max_duration",
               /*description*/
               "The duration of the command, negative values result "
               "in indefinite profiles",
               /*Units*/ "sec")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorProfTorqueService>(
      this, pub_sub_ns_ + "srv/actuator_prof_torque",
      &FcatServices::ActuatorProfTorqueSrvCb, services_qos_, cb_group_blocking_,
      CommandDescriptor(
          "Command a Profiled Torque motion profile to the Actuator. Here, "
          "Torque"
          "is equivalent to Current (frome the DS-402 specification)",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "target_torque_amps",
               /*description*/ "The desired effort in current",
               /*Units*/ "Amps"),
           CommandArgumentDescriptor(
               /*arg name*/ "max_duration",
               /*description*/
               "The duration of the command, "
               "negative values result in indefinite profiles",
               /*Units*/ "sec")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorCalibrateService>(
      this, pub_sub_ns_ + "srv/actuator_calibrate",
      &FcatServices::ActuatorCalibrateSrvCb, services_qos_, cb_group_blocking_,
      CommandDescriptor(
          "Command hardstop calibration to the Actuator. The actuator will "
          "move in "
          "the signed velocity direction with the specified lowered current "
          "limit "
          "to detect the hardstop. From there, the position is set to the "
          "calibration "
          "limit defined in the configuration file. Finally, the original "
          "current is "
          "restored and the actuator is commanded to the command limit defined "
          "in the "
          "configuration file.",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "velocity",
               /*description*/
               "The desired approach velocity for all motions during "
               "hardstop calibration",
               /*Units*/ "EU/sec"),
           CommandArgumentDescriptor(
               /*arg name*/ "accel",
               /*description*/ "The desired acceleration of the motion profile",
               /*Units*/ "EU/sec^2"),
           CommandArgumentDescriptor(
               /*arg name*/ "max_current",
               /*description*/
               "Overrides the Peak current setting but only during "
               "hardstop contact motion",
               /*Units*/ "Amps")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorSetGainSchedulingModeService>(
      this, pub_sub_ns_ + "srv/actuator_set_gain_scheduling_mode",
      &FcatServices::ActuatorSetGainSchedulingModeSrvCb, services_qos_,
      cb_group_blocking_,
      CommandDescriptor(
          "Set the Gain Schedule Mode for the actuator GS[2]. "
          "0 - No Gain Scheduling. "
          "[1,63] - Specific controller from table. "
          "64 - Enable Speed-based scheduling. "
          "65 - Enable position-based scheduling. "
          "66 - Best Settling. "
          "67 - manually scheduled, high-bits. "
          "68 - manually scheduled, low-bits."
          "Use this interface only if you know what you are doing, Caveat "
          "Emptor!",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "gain_scheduling_mode",
               /*description*/ "The desired gain schedule mode value")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorSetUnitModeService>(
      this, pub_sub_ns_ + "srv/actuator_set_unit_mode",
      &FcatServices::ActuatorSetUnitModeSrvCb, services_qos_,
      cb_group_blocking_,
      CommandDescriptor(
          "Sets the Unit Mode UM[1]. "
          "1 - Torque Control. "
          "2 - Speed Control. "
          "3 - Stepper Control. "
          "5 - Position Control (default). "
          "6 - Stepper Open or Closed Loop.",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "gain_scheduling_mode",
               /*description*/ "The desired Unit Mode UM[1]. ")}));

  DeclareServiceCommand<fcat_msgs::srv::PidActivateService>(
      this, pub_sub_ns_ + "srv/pid_activate", &FcatServices::PidActivateSrvCb,
      services_qos_, cb_group_blocking_,
      CommandDescriptor(
          "Start the PID Controller with specified parameters.  "
          "Returns failure if it does not converge in the specified duration. "
          "Returns success and self-disables if the controller converges.",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "setpoint",
               /*description*/ "The controller setpoint",
               /*Units*/ "EU"),
           CommandArgumentDescriptor(
               /*arg name*/ "deadband",
               /*description*/
               "The controller deadband tolerance around the setpoint",
               /*Units*/ "EU"),
           CommandArgumentDescriptor(
               /*arg name*/ "persistence_duration",
               /*description*/
               "The amount of time the signal must remain with the "
               "deadband to end control",
               /*Units*/ "sec"),
           CommandArgumentDescriptor(
               /*arg name*/ "max_duration",
               /*description*/
               "The maximum duration of the command. Beware: "
               "negative values can result in indefinite runtime",
               /*Units*/ "sec")}));

  DeclareServiceCommand<fcat_msgs::srv::AsyncSdoWriteService>(
      this, pub_sub_ns_ + "srv/async_sdo_write",
      &FcatServices::AsyncSdoWriteSrvCb, services_qos_, cb_group_blocking_,
      CommandDescriptor(
          "Issues an Asynchronous SDO parameter Write and waits for the "
          "result of the operation to indicate success or failure.",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "sdo_index",
               /*description*/ "The SDO register index in hexidecimal (0x3034) "
                               "or "
                               "decimal (12340) form"),
           CommandArgumentDescriptor(
               /*arg name*/ "sdo_subindex",
               /*description*/ "The SDO register subindex (e.g. the 1 in "
                               "0x3034:1)"),
           CommandArgumentDescriptor(
               /*arg name*/ "data",
               /*description*/ "Data payload to write to the SDO"),
           CommandArgumentDescriptor(
               /*arg name*/ "data_type",
               /*description*/ "The type of the data payload, must be one of: "
                               "{I8, I16, I32, I64, F32, U8, U16, U32, "
                               "U64}")}));

  DeclareServiceCommand<fcat_msgs::srv::AsyncSdoReadService>(
      this, pub_sub_ns_ + "srv/async_sdo_read",
      &FcatServices::AsyncSdoReadSrvCb, services_qos_, cb_group_blocking_,
      CommandDescriptor(
          "Issues an Asynchronous SDO parameter Read and waits for the "
          "result of the operation to indicate success or failure.",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "sdo_index",
               /*description*/ "The SDO register index in hexidecimal (0x3034) "
                               "or "
                               "decimal (12340) form"),
           CommandArgumentDescriptor(
               /*arg name*/ "sdo_subindex",
               /*description*/ "The SDO register subindex (the 1 in 0x3034:1)"),
           CommandArgumentDescriptor(
               /*arg name*/ "data_type",
               /*description*/ "The type of the data payload, must be one of: "
                               "{I8, I16, I32, I64, F32, U8, U16, U32, "
                               "U64}")}));

  DeclareServiceCommand<fcat_msgs::srv::TlcWriteService>(
      this, pub_sub_ns_ + "srv/tlc_write", &FcatServices::TlcWriteSrvCb,
      services_qos_, cb_group_blocking_,
      CommandDescriptor(
          "Issues a ELMO Two-Letter Command (TLC) which in turn issues an "
          "Asynchronous SDO "
          "parameter Write and waits for the result to indicate success or "
          "failure. Consult "
          "The MAN-G-CR document for full listing of drive data objects.",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "tlc",
               /*description*/ "The Two-Letter Command (TLC) found in the ELMO "
                               "MAN-G-CR Command Reference Document. These two "
                               "chars "
                               "are converted to an SDO index. (e.g. PL, CL)"),
           CommandArgumentDescriptor(
               /*arg name*/ "subindex",
               /*description*/ "The TLC/SDO register subindex (the 1 in "
                               "PL[1])"),
           CommandArgumentDescriptor(
               /*arg name*/ "data",
               /*description*/ "Data payload to write to the drive"),
           CommandArgumentDescriptor(
               /*arg name*/ "data_type",
               /*description*/ "The type of the data payload, must be one of: "
                               "{I8, I16, I32, I64, F32, U8, U16, U32, "
                               "U64}")}));

  DeclareServiceCommand<fcat_msgs::srv::TlcReadService>(
      this, pub_sub_ns_ + "srv/tlc_read", &FcatServices::TlcReadSrvCb,
      services_qos_, cb_group_blocking_,
      CommandDescriptor(
          "Issues a ELMO Two-Letter Command (TLC) which in turn issues an "
          "Asynchronous SDO "
          "parameter Read and waits for the result to indicate success or "
          "failure. Consult "
          "The MAN-G-CR document for full listing of drive data objects.",
          {CommandArgumentDescriptor(
               /*arg name*/ "name",
               /*description*/ "The Fastcat Device Name"),
           CommandArgumentDescriptor(
               /*arg name*/ "tlc",
               /*description*/ "The Two-Letter Command (TLC) found in the ELMO "
                               "MAN-G-CR Command Reference Document. These two "
                               "chars "
                               "are converted to an SDO index. (e.g. PL, CL)"),
           CommandArgumentDescriptor(
               /*arg name*/ "subindex",
               /*description*/ "The TLC/SDO register subindex (the 1 in "
                               "PL[1])"),
           CommandArgumentDescriptor(
               /*arg name*/ "data_type",
               /*description*/ "The type of the data payload, must be one of: "
                               "{I8, I16, I32, I64, F32, U8, U16, U32, "
                               "U64}")}));
}

//
// Fcat State Subscription Callbacks
//

void FcatServices::FcatModuleStateCb(
    const std::shared_ptr<fcat_msgs::msg::ModuleState> msg)
{
  module_state_last_recv_time_ = this->now().seconds();
  fcat_module_state_msg_       = *msg;
}

void FcatServices::ActuatorStatesCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorStates> msg)
{
  act_states_last_recv_time_ = this->now().seconds();
  actuator_states_msg_       = *msg;

  for (size_t i = 0; i < actuator_states_msg_.names.size(); ++i) {
    act_state_map_[actuator_states_msg_.names[i]] =
        actuator_states_msg_.states[i];
  }
}

void FcatServices::PidStatesCb(
    const std::shared_ptr<fcat_msgs::msg::PidStates> msg)
{
  pid_states_last_recv_time_ = this->now().seconds();
  pid_states_msg_            = *msg;

  for (size_t i = 0; i < pid_states_msg_.names.size(); ++i) {
    pid_state_map_[pid_states_msg_.names[i]] = pid_states_msg_.states[i];
  }
}

void FcatServices::AsyncSdoResponseCb(
    const std::shared_ptr<fcat_msgs::msg::AsyncSdoResponse> msg)
{
  // prevent the queue from filling up too much
  if (sdo_response_queue_.size() >= max_sdo_queue_size_) {
    sdo_response_queue_.pop();
  }
  sdo_response_queue_.push((*msg));
}

//
// Helper Functions
//
bool FcatServices::ActuatorCmdPrechecks(std::string name, std::string &message)
{
  // //1) Check fcat module state liveliness check
  // if((get_time_sec() - module_state_last_recv_time_) > FCAT_services_LIVELINESS_DURATION){ 
  //   message = "stale fcat module state topic, fcat is not running";
  //   WARNING("bad command: %s", message.c_str());
  //   return false;
  // }

  // //2) Check fcat module is not faulted
  // if(fcat_module_state_msg_.faulted){
  //   message = "fcat is faulted, reset fcat first";
  //   WARNING("bad command: %s", message.c_str());
  //   return false;
  // }

  // //3) Check Actuator States liveliness check
  // if((get_time_sec() - act_states_last_recv_time_) > FCAT_services_LIVELINESS_DURATION){ 
  //   message = "stale fcat actuator states topic, check fastcat YAML has any actuators";
  //   WARNING("bad command: %s", message.c_str());
  //   return false;
  // }

  // //4) Check Actuator name is on the bus
  // if(act_state_map_.find(name) == act_state_map_.end()){
  //   message = "actuator name not found, check device name";
  //   WARNING("bad command: %s", message.c_str());
  //   return false;
  // }
  // return true;
  // 1) Check fcat module state liveliness check
  if ((this->now().seconds() - module_state_last_recv_time_) >
      liveliness_duration_sec_) {
    message = "stale fcat module state topic, fcat is not running";
    MSG("bad command: %s", message.c_str());
    return false;
  }

  // 2) Check fcat module is not faulted
  if (fcat_module_state_msg_.faulted) {
    message = "fcat is faulted, reset fcat first";
    MSG("bad command: %s", message.c_str());
    return false;
  }

  // 3) Check Actuator States liveliness check
  if ((this->now().seconds() - act_states_last_recv_time_) >
      liveliness_duration_sec_) {
    message =
        "stale fcat actuator states topic, check fastcat YAML has any "
        "actuators";
    MSG("bad command: %s", message.c_str());
    return false;
  }

  // 4) Check Actuator name is on the bus
  if (act_state_map_.find(name) == act_state_map_.end()) {
    message = "actuator name not found, check device name";
    MSG("bad command: %s", message.c_str());
    return false;
  }
  return true;
}

bool FcatServices::PidCmdPrechecks(std::string name, std::string &message){
  // //1) Check fcat module state liveliness check
  // if((get_time_sec() - module_state_last_recv_time_) > FCAT_services_LIVELINESS_DURATION){ 
  //   message = "stale fcat module state topic, fcat is not running";
  //   WARNING("bad command: %s", message.c_str());
  //   return false;
  // }

  // //2) Check fcat module is not faulted
  // if(fcat_module_state_msg_.faulted){
  //   message = "fcat is faulted, reset fcat first";
  //   WARNING("bad command: %s", message.c_str());
  //   return false;
  // }

  // //3) Check Pid States liveliness check
  // if((get_time_sec() - pid_states_last_recv_time_) > FCAT_services_LIVELINESS_DURATION){ 
  //   message = "stale fcat pid states topic, check fastcat YAML has any pid devices";
  //   WARNING("bad command: %s", message.c_str());
  //   return false;
  // }

  // //4) Check Pid name is on the bus
  // if(pid_state_map_.find(name) == pid_state_map_.end()){
  //   message = "pid name not found, check device name";
  //   WARNING("bad command: %s", message.c_str());
  //   return false;
  // }
  // return true;
  // 1) Check fcat module state liveliness check
  if ((this->now().seconds() - module_state_last_recv_time_) >
      liveliness_duration_sec_) {
    message = "stale fcat module state topic, fcat is not running";
    MSG("bad command: %s", message.c_str());
    return false;
  }

  // 2) Check fcat module is not faulted
  if (fcat_module_state_msg_.faulted) {
    message = "fcat is faulted, reset fcat first";
    MSG("bad command: %s", message.c_str());
    return false;
  }

  // 3) Check Pid States liveliness check
  if ((this->now().seconds() - pid_states_last_recv_time_) >
      liveliness_duration_sec_) {
    message =
        "stale fcat pid states topic, check fastcat YAML has any pid devices";
    MSG("bad command: %s", message.c_str());
    return false;
  }

  // 4) Check Pid name is on the bus
  if (pid_state_map_.find(name) == pid_state_map_.end()) {
    message = "pid name not found, check device name";
    MSG("bad command: %s", message.c_str());
    return false;
  }
  return true;
}

bool FcatServices::CheckCommonFaultActive(std::string& error_message)
{
  if (!rclcpp::ok()) {
    error_message = "fcat_srvs node shutdown";
    MSG("Failure: %s", error_message.c_str());
    return true;
  }

  if ((this->now().seconds() - module_state_last_recv_time_) >
      liveliness_duration_sec_) {
    error_message = "stale fcat module state topic, fcat has stopped ";
    MSG("Failure: %s", error_message.c_str());
    return true;
  }

  if (fcat_module_state_msg_.faulted) {
    error_message = "fcat is encountered a fault";
    MSG("%s", error_message.c_str());
    return false;
  }

  return false;
}

bool FcatServices::RunActuatorMonitorLoop(std::string& error_message,
                                          std::string  act_name)
{
  fastcat::ActuatorStateMachineState sms;
  srv_state_ = FCAT_SRV_STATE_IDLE_CHECKING;

  int64_t idle_persist_rti = this->get_parameter("idle_persist_rti").as_int();
  int64_t idle_rti_count   = 0;

  // Continuously check for
  //  - fcat_srvs shutdown request
  //  - fcat faults
  //  - actuator state machine progress

  while (true) {
    rate_->sleep();

    if (CheckCommonFaultActive(error_message)) {
      return false;
    }

    sms = static_cast<fastcat::ActuatorStateMachineState>(
        act_state_map_[act_name].actuator_state_machine_state);

    if (sms == fastcat::ACTUATOR_SMS_FAULTED) {
      error_message = std::string("Actuator unexpectedly faulted");
      MSG("Failure: %s", error_message.c_str());
      return false;
    }

    switch (srv_state_) {
      case FCAT_SRV_STATE_IDLE_CHECKING:

        // In the event that the command is trivial, the actuator state machine
        // may not change in response to a queued command sent to the fastcat
        // manager. This check responds "success" if no state machine change is
        // detected
        if (sms == fastcat::ACTUATOR_SMS_HALTED ||
            sms == fastcat::ACTUATOR_SMS_HOLDING) {
          if (idle_rti_count >= idle_persist_rti) {
            MSG(
                "Max number of RTIs (%ld) elapsed with no Actuator SMS change, "
                "interpreting this result as a success",
                idle_persist_rti);

            return true;
          }
          idle_rti_count++;
        } else {
          // must be in motion by now (faulted would be caught above)
          srv_state_ = FCAT_SRV_STATE_RUNNING;
        }
        break;

      case FCAT_SRV_STATE_RUNNING:
        if (sms == fastcat::ACTUATOR_SMS_HALTED ||
            sms == fastcat::ACTUATOR_SMS_HOLDING) {
          return true;
        }
        break;

      default:
        MSG("bad srv state, aborting command");
        return false;
    }
  }

  return true;
}

bool FcatServices::WaitForSdoResponse(std::string& message, uint16_t app_id)
{
  char   print_str[512];
  double sdo_wait_duration_sec =
      this->get_parameter("sdo_wait_duration_sec").as_double();
  double start_time = this->now().seconds();
  while (this->now().seconds() < (sdo_wait_duration_sec + start_time)) {
    // Check for FCAT fault
    if (CheckCommonFaultActive(message)) {
      return false;
    }

    if (!sdo_response_queue_.empty()) {
      async_sdo_response_msg_ = sdo_response_queue_.front();
      sdo_response_queue_.pop();

      // Check the app id, discard and keep waiting if it does not match
      // if (app_id != async_sdo_response_msg_.app_id) {
      //   CFW_DEBUG(
      //       "SDO Response received but actual app_id:(%u) does not match "
      //       "expected app_id:(%u). Continuing to wait...",
      //       async_sdo_response_msg_.app_id, app_id);
      //   continue;
      // }

      bool success = async_sdo_response_msg_.success;

      if (!success) {
        sprintf(print_str,
                "SDO Response Failed {Name:(%s) Index:(0x%x)"
                " Subindex:(%u) Data:(%s) DataType:(%s) AppId:(%u)}",
                async_sdo_response_msg_.device_name.c_str(),
                async_sdo_response_msg_.sdo_index,
                async_sdo_response_msg_.sdo_subindex,
                async_sdo_response_msg_.data.c_str(),
                async_sdo_response_msg_.data_type.c_str(),
                async_sdo_response_msg_.app_id);
        message = print_str;
        MSG("%s", print_str);
      } else {
        sprintf(print_str,
                "SDO Response Success {Name:(%s) Index:(0x%x)"
                " Subindex:(%u) Data:(%s) DataType:(%s) AppId:(%u)}",
                async_sdo_response_msg_.device_name.c_str(),
                async_sdo_response_msg_.sdo_index,
                async_sdo_response_msg_.sdo_subindex,
                async_sdo_response_msg_.data.c_str(),
                async_sdo_response_msg_.data_type.c_str(),
                async_sdo_response_msg_.app_id);
        message = print_str;
        MSG("%s", print_str);
      }
      return success;
    }

    rate_->sleep();
  }

  // if here, runout timer has expired
  sprintf(print_str,
          "SDO Runout timer expired for app_id:(%u). "
          "Parameter 'sdo_wait_duration_sec' is set to (%lf)",
          app_id, sdo_wait_duration_sec);
  message = print_str;
  MSG("%s", print_str);
  return false;
}

// void FcatServices::FcatModuleStateCb(
//     const std::shared_ptr<fcat_msgs::msg::ModuleState> msg)
// {
//   module_state_last_recv_time_ = get_time_sec();
//   fcat_module_state_msg_ = *msg;
// }

// void FcatServices::ActuatorStatesCb(
//     const std::shared_ptr<fcat_msgs::msg::ActuatorStates> msg)
// {
//   act_states_last_recv_time_ = get_time_sec();
//   actuator_states_msg_ = *msg;

//   for(size_t i = 0; i < actuator_states_msg_.names.size(); ++i){
//     act_state_map_[actuator_states_msg_.names[i]] = actuator_states_msg_.states[i];
//   }
// }

// void FcatServices::PidStatesCb(
//     const std::shared_ptr<fcat_msgs::msg::PidStates> msg)
// {
//   pid_states_last_recv_time_ = get_time_sec();
//   pid_states_msg_ = *msg;

//   for(size_t i = 0; i < pid_states_msg_.names.size(); ++i){
//     pid_state_map_[pid_states_msg_.names[i]] = pid_states_msg_.states[i];
//   }
// }

// void FcatServices::ActuatorProfPosSrvCb(
//     const std::shared_ptr<fcat_msgs::srv::ActuatorProfPosCmd::Request> request,
//     std::shared_ptr<fcat_msgs::srv::ActuatorProfPosCmd::Response>      response)
// {
//   MSG("fcat_srv recived new actuator_prof_pos command");
//   response->success = false;

//   if(!ActuatorCmdPrechecks(request->name, response->message)){
//     return;
//   }

//   MSG("actuator_prof_pos: issuing topic cmd and watching for completion");

//   //Publish topic to start the service
//   auto cmd_msg = fcat_msgs::msg::ActuatorProfPosCmd();

//   cmd_msg.name             = request->name;
//   cmd_msg.target_position  = request->target_position;
//   cmd_msg.profile_velocity = request->profile_velocity;
//   cmd_msg.profile_accel    = request->profile_accel;
//   cmd_msg.relative         = request->relative;

//   act_prof_pos_pub_->publish(cmd_msg);


//   // Continuously check for 
//   //  - fcat_services shutdown request
//   //  - fcat faults
//   //  - actuator state machine progress

//   fastcat::ActuatorStateMachineState sms;

//   double pos;
//   rclcpp::Rate timer(loop_rate_hz_); //Hz

//   while(true){
//     if(!rclcpp::ok()){
//       response->message = "fcat_services node shutdown";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     if((get_time_sec() - module_state_last_recv_time_) > FCAT_services_LIVELINESS_DURATION){ 
//       response->message = "stale fcat module state topic, fcat has stopped ";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     sms = static_cast<fastcat::ActuatorStateMachineState>(
//         act_state_map_[request->name].actuator_state_machine_state);
//     pos = act_state_map_[request->name].actual_position;

//     if(sms == fastcat::ACTUATOR_SMS_FAULTED){
//       response->message = std::string("Actuator unexpectedly faulted");
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }


//     if(sms != fastcat::ACTUATOR_SMS_PROF_POS &&
//         fabs(pos - request->target_position) < position_tolerance_){
//       break;
//     }

//     timer.sleep();
//   }

//   MSG("Completed ACTUATOR_PROF_POS for: %s", request->name.c_str());
//   response->message = "";
//   response->success = true;
// }

// void FcatServices::ActuatorProfVelSrvCb(
//     const std::shared_ptr<fcat_msgs::srv::ActuatorProfVelCmd::Request> request,
//     std::shared_ptr<fcat_msgs::srv::ActuatorProfVelCmd::Response>      response)
// {
//   MSG("fcat_srv recived new actuator_prof_vel command");
//   response->success = false;

//   if(!ActuatorCmdPrechecks(request->name, response->message)){
//     return;
//   }

//   MSG("actuator_prof_vel: issuing topic cmd and watching for completion");

//   //Publish topic to start the service
//   auto cmd_msg = fcat_msgs::msg::ActuatorProfVelCmd();

//   cmd_msg.name            = request->name;
//   cmd_msg.target_velocity = request->target_velocity;
//   cmd_msg.profile_accel   = request->profile_accel;
//   cmd_msg.max_duration    = request->max_duration;

//   act_prof_vel_pub_->publish(cmd_msg);

//   // Continuously check for 
//   //  - fcat_services shutdown request
//   //  - fcat faults
//   //  - actuator state machine progress

//   fastcat::ActuatorStateMachineState sms;
//   double velocity;
//   rclcpp::Rate timer(loop_rate_hz_); //Hz

//   while(true){
//     if(!rclcpp::ok()){
//       response->message = "fcat_services node shutdown";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     if((get_time_sec() - module_state_last_recv_time_) > FCAT_services_LIVELINESS_DURATION){ 
//       response->message = "stale fcat module state topic, fcat has stopped ";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     sms = static_cast<fastcat::ActuatorStateMachineState>(
//         act_state_map_[request->name].actuator_state_machine_state);

//     velocity = act_state_map_[request->name].cmd_velocity;

//     if(sms == fastcat::ACTUATOR_SMS_FAULTED){
//       response->message = std::string("Actuator unexpectedly faulted");
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     if(sms == fastcat::ACTUATOR_SMS_PROF_VEL && 
//         fabs(velocity - request->target_velocity) < velocity_tolerance_){
//       MSG("Actuator is in PROF_VEL mode and achieved target velocity");
//       break;
//     }

//     timer.sleep();
//   }

//   MSG("Completed ACTUATOR_PROF_VEL for: %s", request->name.c_str());
//   response->message = "";
//   response->success = true;
// }

// void FcatServices::ActuatorProfTorqueSrvCb(
//     const std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueCmd::Request> request,
//     std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueCmd::Response>      response)
// {
//   MSG("fcat_srv recived new actuator_prof_torque command");
//   response->success = false;

//   if(!ActuatorCmdPrechecks(request->name, response->message)){
//     return;
//   }

//   MSG("actuator_prof_torque: issuing topic cmd and watching for completion");

//   //Publish topic to start the service
//   auto cmd_msg = fcat_msgs::msg::ActuatorProfTorqueCmd();

//   cmd_msg.name               = request->name;
//   cmd_msg.target_torque_amps = request->target_torque_amps;
//   cmd_msg.max_duration       = request->max_duration;

//   act_prof_torque_pub_->publish(cmd_msg);

//   // Continuously check for 
//   //  - fcat_services shutdown request
//   //  - fcat faults
//   //  - actuator state machine progress

//   fastcat::ActuatorStateMachineState sms;
//   double current;
//   rclcpp::Rate timer(loop_rate_hz_); //Hz

//   while(true){
//     if(!rclcpp::ok()){
//       response->message = "fcat_services node shutdown";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     if((get_time_sec() - module_state_last_recv_time_) > FCAT_services_LIVELINESS_DURATION){ 
//       response->message = "stale fcat module state topic, fcat has stopped ";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     sms = static_cast<fastcat::ActuatorStateMachineState>(
//         act_state_map_[request->name].actuator_state_machine_state);

//     current = act_state_map_[request->name].cmd_current;

//     if(sms == fastcat::ACTUATOR_SMS_FAULTED){
//       response->message = std::string("Actuator unexpectedly faulted");
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     if(sms == fastcat::ACTUATOR_SMS_PROF_TORQUE && 
//         fabs(current - request->target_torque_amps) < current_tolerance_){
//       MSG("Actuator is in PROF_TORQUE mode and achieved target current");
//       break;
//     }

//     timer.sleep();
//   }

//   MSG("Completed ACTUATOR_PROF_TORQUE for: %s", request->name.c_str());
//   response->message = "";
//   response->success = true;
// }

// void FcatServices::ActuatorCalibrateSrvCb(
//     const std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateCmd::Request> request,
//     std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateCmd::Response>      response)
// {
//   MSG("fcat_srv recived new actuator_calibrate command");
//   response->success = false;

//   if(!ActuatorCmdPrechecks(request->name, response->message)){
//     return;
//   }

//   MSG("actuator_calibrate: issuing topic cmd and watching for completion");

//   //Publish topic to start the service
//   auto cmd_msg = fcat_msgs::msg::ActuatorCalibrateCmd();

//   cmd_msg.name        = request->name;
//   cmd_msg.velocity    = request->velocity;
//   cmd_msg.accel       = request->accel;
//   cmd_msg.max_current = request->max_current;

//   act_calibrate_pub_->publish(cmd_msg);

//   // Continuously check for 
//   //  - fcat_services shutdown request
//   //  - fcat faults
//   //  - actuator state machine progress

//   fastcat::ActuatorStateMachineState sms;
//   fastcat::ActuatorStateMachineState last_sms;
//   last_sms = static_cast<fastcat::ActuatorStateMachineState>(
//       act_state_map_[request->name].actuator_state_machine_state);

//   rclcpp::Rate timer(loop_rate_hz_); //Hz

//   while(true){
//     if(!rclcpp::ok()){
//       response->message = "fcat_services node shutdown";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     if((get_time_sec() - module_state_last_recv_time_) > FCAT_services_LIVELINESS_DURATION){ 
//       response->message = "stale fcat module state topic, fcat has stopped ";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     sms = static_cast<fastcat::ActuatorStateMachineState>(
//         act_state_map_[request->name].actuator_state_machine_state);

//     if(sms == fastcat::ACTUATOR_SMS_FAULTED){
//       response->message = std::string("Actuator unexpectedly faulted");
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     if(sms != fastcat::ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP && 
//          last_sms == fastcat::ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP){
//       break;
//     }

//     last_sms = sms;
//     timer.sleep();
//   }

//   MSG("Completed ACTUATOR_CALIBRATE for: %s", request->name.c_str());
//   response->message = "";
//   response->success = true;
// }

// void FcatServices::PidActivateSrvCb(
//     const std::shared_ptr<fcat_msgs::srv::PidActivateCmd::Request> request,
//     std::shared_ptr<fcat_msgs::srv::PidActivateCmd::Response>      response)
// {
//   MSG("fcat_srv recived new pid_activate command");
//   response->success = false;

//   if(!PidCmdPrechecks(request->name, response->message)){
//     return;
//   }

//   MSG("pid_activate: issuing topic cmd and watching for completion");

//   //Publish topic to start the service
//   auto cmd_msg = fcat_msgs::msg::PidActivateCmd();

//   cmd_msg.name                 = request->name;
//   cmd_msg.setpoint             = request->setpoint;
//   cmd_msg.deadband             = request->deadband;
//   cmd_msg.persistence_duration = request->persistence_duration;
//   cmd_msg.max_duration         = request->max_duration;

//   pid_activate_pub_->publish(cmd_msg);

//   // Continuously check for 
//   //  - fcat_services shutdown request
//   //  - fcat faults
//   //  - pid active flag
//   bool pid_is_active;
//   bool last_pid_is_active = pid_state_map_[request->name].active;

//   rclcpp::Rate timer(loop_rate_hz_); //Hz

//   while(true){
//     if(!rclcpp::ok()){
//       response->message = "fcat_services node shutdown";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     if((get_time_sec() - module_state_last_recv_time_) > FCAT_services_LIVELINESS_DURATION){ 
//       response->message = "stale fcat module state topic, fcat has stopped ";
//       ERROR("Failure: %s", response->message.c_str());
//       return;
//     }

//     pid_is_active = pid_state_map_[request->name].active;

//     if(!pid_is_active && last_pid_is_active){
//       break;
//     }

//     last_pid_is_active = pid_is_active;
//     timer.sleep();
//   }

//   MSG("Completed ACTUATOR_PROF_TORQUE for: %s", request->name.c_str());
//   response->message = "";
//   response->success = true;
// }

//
// Service Callbacks
//

void FcatServices::ActuatorProfPosSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorProfPosService::Request>
                                                                      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorProfPosService::Response> response)
{
  MSG("Handling Actuator Prof Pos Command");
  response->success = false;

  if (!ActuatorCmdPrechecks(request->name, response->message)) {
    return;
  }

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorProfPosCmd();

  cmd_msg.name             = request->name;
  cmd_msg.target_position  = request->target_position;
  cmd_msg.profile_velocity = request->profile_velocity;
  cmd_msg.profile_accel    = request->profile_accel;
  cmd_msg.relative         = request->relative;

  act_prof_pos_pub_->publish(cmd_msg);

  double goal_pos = request->target_position;
  if (request->relative) {
    goal_pos += act_state_map_[request->name].actual_position;
  }

  MSG("actuator:%s Profile Position command from: %lf to goal: %lf",
                  request->name.c_str(),
                  act_state_map_[request->name].actual_position, goal_pos);

  response->success = RunActuatorMonitorLoop(response->message, request->name);
  if (response->success) {
    MSG("Completed ACTUATOR_PROF_POS for: %s",
                    request->name.c_str());
    response->message = "";  // not strictly needed
  }
}

void FcatServices::ActuatorProfVelSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorProfVelService::Request>
                                                                      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorProfVelService::Response> response)
{
  MSG("Handling Actuator Prof Vel Command");
  response->success = false;

  if (!ActuatorCmdPrechecks(request->name, response->message)) {
    return;
  }

  double tol = this->get_parameter("tolerance").as_double();
  if (fabs(request->max_duration) < tol) {
    MSG(
        "Max Duration (%E) < tolerance (%E) indicating this runs forever. "
        "In this case, use the topic (non-blocking) interface instead of the "
        "service (blocking) interface",
        request->max_duration, tol);
    return;
  }

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorProfVelCmd();

  cmd_msg.name            = request->name;
  cmd_msg.target_velocity = request->target_velocity;
  cmd_msg.profile_accel   = request->profile_accel;
  cmd_msg.max_duration    = request->max_duration;

  act_prof_vel_pub_->publish(cmd_msg);

  MSG("actuator:%s to goal velocity value: %lf",
                  request->name.c_str(), request->target_velocity);

  response->success = RunActuatorMonitorLoop(response->message, request->name);
  if (response->success) {
    MSG("Completed ACTUATOR_PROF_VEL for: %s",
                    request->name.c_str());
    response->message = "";
  }
}

void FcatServices::ActuatorProfTorqueSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueService::Request>
        request,
    std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueService::Response>
        response)
{
  MSG("Handling Actuator Prof Torque Command");
  response->success = false;

  if (!ActuatorCmdPrechecks(request->name, response->message)) {
    return;
  }

  MSG(
      "actuator_prof_torque: issuing topic cmd and watching for completion");

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorProfTorqueCmd();

  cmd_msg.name               = request->name;
  cmd_msg.target_torque_amps = request->target_torque_amps;
  cmd_msg.max_duration       = request->max_duration;

  act_prof_torque_pub_->publish(cmd_msg);

  MSG("fcat_srv actuator:%s to goal current value: %lf",
                  request->name.c_str(), request->target_torque_amps);

  response->success = RunActuatorMonitorLoop(response->message, request->name);
  if (response->success) {
    MSG("Completed ACTUATOR_PROF_TORQUE for: %s",
                    request->name.c_str());
    response->message = "";
  }
}

void FcatServices::ActuatorCalibrateSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Request>
        request,
    std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Response>
        response)
{
  MSG("Handling Actuator Calibrate Command");
  response->success = false;

  if (!ActuatorCmdPrechecks(request->name, response->message)) {
    return;
  }

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorCalibrateCmd();

  cmd_msg.name        = request->name;
  cmd_msg.velocity    = request->velocity;
  cmd_msg.accel       = request->accel;
  cmd_msg.max_current = request->max_current;

  act_calibrate_pub_->publish(cmd_msg);

  response->success = RunActuatorMonitorLoop(response->message, request->name);
  if (response->success) {
    MSG("Completed ACTUATOR_CALIBRATE for: %s",
                    request->name.c_str());
    response->message = "";
  }
}

void FcatServices::ActuatorSetGainSchedulingModeSrvCb(
    const std::shared_ptr<
        fcat_msgs::srv::ActuatorSetGainSchedulingModeService::Request>
        request,
    std::shared_ptr<
        fcat_msgs::srv::ActuatorSetGainSchedulingModeService::Response>
        response)
{
  MSG("Handling Actuator Set Gain Scheduling Mode Command");
  response->success = false;

  auto tlc_req = std::make_shared<fcat_msgs::srv::TlcWriteService::Request>();
  auto tlc_res = std::make_shared<fcat_msgs::srv::TlcWriteService::Response>();

  tlc_req->name      = request->name;
  tlc_req->tlc       = "GS";
  tlc_req->subindex  = 2;
  tlc_req->data      = std::to_string(request->gain_scheduling_mode);
  tlc_req->data_type = "I32";

  TlcWriteSrvCb(tlc_req, tlc_res);

  response->message = tlc_res->message;
  response->success = tlc_res->success;
}

void FcatServices::ActuatorSetUnitModeSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorSetUnitModeService::Request>
        request,
    std::shared_ptr<fcat_msgs::srv::ActuatorSetUnitModeService::Response>
        response)
{
  MSG("Handling Actuator Set Unit Mode Command");
  response->success = false;

  auto tlc_req = std::make_shared<fcat_msgs::srv::TlcWriteService::Request>();
  auto tlc_res = std::make_shared<fcat_msgs::srv::TlcWriteService::Response>();

  tlc_req->name      = request->name;
  tlc_req->tlc       = "UM";
  tlc_req->subindex  = 1;
  tlc_req->data      = std::to_string(request->mode);
  tlc_req->data_type = "I32";

  TlcWriteSrvCb(tlc_req, tlc_res);

  response->message = tlc_res->message;
  response->success = tlc_res->success;
}

void FcatServices::PidActivateSrvCb(
    const std::shared_ptr<fcat_msgs::srv::PidActivateService::Request> request,
    std::shared_ptr<fcat_msgs::srv::PidActivateService::Response>      response)
{
  MSG("Handling Pid Activate Command");
  response->success = false;

  if (!PidCmdPrechecks(request->name, response->message)) {
    return;
  }

  MSG("pid_activate: issuing topic cmd and watching for completion");

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::PidActivateCmd();

  cmd_msg.name                 = request->name;
  cmd_msg.setpoint             = request->setpoint;
  cmd_msg.deadband             = request->deadband;
  cmd_msg.persistence_duration = request->persistence_duration;
  cmd_msg.max_duration         = request->max_duration;

  pid_activate_pub_->publish(cmd_msg);

  // Continuously check for
  //  - fcat_srvs shutdown request
  //  - fcat faults
  //  - pid active flag
  bool pid_is_active;
  bool last_pid_is_active = pid_state_map_[request->name].active;

  while (true) {
    if (CheckCommonFaultActive(response->message)) {
      return;
    }

    pid_is_active = pid_state_map_[request->name].active;

    // This logic may be a little brittle if the
    // command uses low or zero persistence. If an issue is ever
    // reported, solve it here just like for the actuator case.
    if (!pid_is_active && last_pid_is_active) {
      break;
    }

    last_pid_is_active = pid_is_active;
    rate_->sleep();
  }

  MSG("Completed PID_ACTIVATE for: %s", request->name.c_str());
  response->message = "";
  response->success = true;
}

void FcatServices::AsyncSdoWriteSrvCb(
    const std::shared_ptr<fcat_msgs::srv::AsyncSdoWriteService::Request>
                                                                    request,
    std::shared_ptr<fcat_msgs::srv::AsyncSdoWriteService::Response> response)
{
  MSG("Handling SDO Write command");
  char print_str[512];
  response->message = "";

  // 1) Parse input args
  uint16_t parsed_sdo_index = 0;
  if (!HexOrDecStrToNum(request->sdo_index, parsed_sdo_index)) {
    sprintf(print_str, "Invalid SDO index string:(%s) conversion to U16",
            request->sdo_index.c_str());

    response->message = print_str;
    MSG("%s", print_str);

    return;
  }

  if (JSD_SDO_DATA_UNSPECIFIED ==
      jsd_sdo_data_type_from_string(request->data_type)) {
    sprintf(print_str,
            "Invalid SDO Write data_type:(%s) Must be one of "
            "{'I8', 'I16', 'I32', 'I64', 'F32', 'U8', 'U16', 'U32', 'U64'}",
            request->data_type.c_str());
    response->message = print_str;
    MSG("%s", print_str);
    return;
  }

  // 2) Dispatch the SDO request to FCAT
  fcat_msgs::msg::AsyncSdoWriteCmd msg;

  msg.name         = request->name;
  msg.sdo_index    = parsed_sdo_index;
  msg.sdo_subindex = request->sdo_subindex;
  msg.data         = request->data;
  msg.data_type    = request->data_type;
  msg.app_id       = sdo_app_id_++;

  async_sdo_write_pub_->publish(msg);

  MSG(
      "Async SDO Write issued {Name:(%s) Index:(0x%x)"
      " Subindex:(%u) Data:(%s) DataType:(%s) AppId:(%u)}",
      msg.name.c_str(), msg.sdo_index, msg.sdo_subindex, msg.data.c_str(),
      msg.data_type.c_str(), msg.app_id);

  // 3) Wait for the SDO Response from FCAT
  response->success = WaitForSdoResponse(response->message, msg.app_id);
}

void FcatServices::AsyncSdoReadSrvCb(
    const std::shared_ptr<fcat_msgs::srv::AsyncSdoReadService::Request> request,
    std::shared_ptr<fcat_msgs::srv::AsyncSdoReadService::Response> response)
{
  MSG("Handling SDO Read command");
  char print_str[512];
  response->message = "";

  // 1) Parse input args
  uint16_t parsed_sdo_index = 0;
  if (!HexOrDecStrToNum(request->sdo_index, parsed_sdo_index)) {
    sprintf(print_str, "Invalid SDO index string:(%s) conversion to U16",
            request->sdo_index.c_str());

    response->message = print_str;
    MSG("%s", print_str);
    return;
  }

  if (JSD_SDO_DATA_UNSPECIFIED ==
      jsd_sdo_data_type_from_string(request->data_type)) {
    sprintf(print_str,
            "Invalid SDO Read data_type:(%s) Must be one of "
            "{'I8', 'I16', 'I32', 'I64', 'F32', 'U8', 'U16', 'U32', 'U64'}",
            request->data_type.c_str());

    response->message = print_str;
    MSG("%s", print_str);
    return;
  }

  // 2) Dispatch the SDO request to FCAT
  fcat_msgs::msg::AsyncSdoReadCmd msg;

  msg.name         = request->name;
  msg.sdo_index    = parsed_sdo_index;
  msg.sdo_subindex = request->sdo_subindex;
  msg.data_type    = request->data_type;
  msg.app_id       = sdo_app_id_++;

  async_sdo_read_pub_->publish(msg);

  MSG(
      "Async SDO Read issued {Name:(%s) Index:(0x%x)"
      " Subindex:(%u) DataType:(%s) AppId:(%u)}",
      msg.name.c_str(), msg.sdo_index, msg.sdo_subindex, msg.data_type.c_str(),
      msg.app_id);

  // 3) Wait for the SDO Response from FCAT
  response->success = WaitForSdoResponse(response->message, msg.app_id);
}

void FcatServices::TlcWriteSrvCb(
    const std::shared_ptr<fcat_msgs::srv::TlcWriteService::Request> request,
    std::shared_ptr<fcat_msgs::srv::TlcWriteService::Response>      response)
{
  MSG("Handling Two-Letter Command (TLC) Write command");
  char print_str[512];
  response->message = "";

  // 1) Parse input args
  uint16_t parsed_sdo_index = 0;
  if (!TlcStrToNum(request->tlc, parsed_sdo_index)) {
    sprintf(print_str, "Invalid TLC string:(%s) conversion to SDO",
            request->tlc.c_str());

    response->message = print_str;
    MSG("%s", print_str);
    return;
  }

  if (JSD_SDO_DATA_UNSPECIFIED ==
      jsd_sdo_data_type_from_string(request->data_type)) {
    sprintf(print_str,
            "Invalid SDO Write data_type:(%s) Must be one of "
            "{'I8', 'I16', 'I32', 'I64', 'F32', 'U8', 'U16', 'U32', 'U64'}",
            request->data_type.c_str());
    response->message = print_str;
    MSG("%s", print_str);

    return;
  }

  // 2) Dispatch the SDO request to FCAT
  fcat_msgs::msg::AsyncSdoWriteCmd msg;

  msg.name         = request->name;
  msg.sdo_index    = parsed_sdo_index;
  msg.sdo_subindex = request->subindex;
  msg.data         = request->data;
  msg.data_type    = request->data_type;
  msg.app_id       = sdo_app_id_++;

  async_sdo_write_pub_->publish(msg);

  MSG(
      "Async SDO Write issued {Name:(%s) TLC:(%s) Index:(0x%x)"
      " Subindex:(%u) Data:(%s) DataType:(%s) AppId:(%u)}",
      msg.name.c_str(), request->tlc.c_str(), msg.sdo_index, msg.sdo_subindex,
      msg.data.c_str(), msg.data_type.c_str(), msg.app_id);

  // 3) Wait for the SDO Response from FCAT
  response->success = WaitForSdoResponse(response->message, msg.app_id);
}

void FcatServices::TlcReadSrvCb(
    const std::shared_ptr<fcat_msgs::srv::TlcReadService::Request> request,
    std::shared_ptr<fcat_msgs::srv::TlcReadService::Response>      response)
{
  MSG("Handling Two-Letter Command (TLC) Read command");
  char print_str[512];
  response->message = "";

  // 1) Parse input args
  uint16_t parsed_sdo_index = 0;
  if (!TlcStrToNum(request->tlc, parsed_sdo_index)) {
    sprintf(print_str, "Invalid TLC string:(%s) conversion to SDO",
            request->tlc.c_str());

    response->message = print_str;
    MSG("%s", print_str);
    return;
  }

  if (JSD_SDO_DATA_UNSPECIFIED ==
      jsd_sdo_data_type_from_string(request->data_type)) {
    sprintf(print_str,
            "Invalid SDO Read data_type:(%s) Must be one of "
            "{'I8', 'I16', 'I32', 'I64', 'F32', 'U8', 'U16', 'U32', 'U64'}",
            request->data_type.c_str());

    response->message = print_str;
    MSG("%s", print_str);
    return;
  }

  // 2) Dispatch the SDO request to FCAT
  fcat_msgs::msg::AsyncSdoReadCmd msg;

  msg.name         = request->name;
  msg.sdo_index    = parsed_sdo_index;
  msg.sdo_subindex = request->subindex;
  msg.data_type    = request->data_type;
  msg.app_id       = sdo_app_id_++;

  async_sdo_read_pub_->publish(msg);

  MSG(
      "Async SDO Read issued {Name:(%s) TLC:(%s) Index:(0x%x)"
      " Subindex:(%u) DataType:(%s) AppId:(%u)}",
      msg.name.c_str(), request->tlc.c_str(), msg.sdo_index, msg.sdo_subindex,
      msg.data_type.c_str(), msg.app_id);

  // 3) Wait for the SDO Response from FCAT
  response->success = WaitForSdoResponse(response->message, msg.app_id);
}
