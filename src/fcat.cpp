#include "fcat/fcat.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sched.h>
#include <unistd.h>
#include <sys/sysinfo.h>

#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

Fcat::~Fcat(){
  fcat_manager_.Shutdown();
}

Fcat::Fcat(const rclcpp::NodeOptions& options) : FcatNode("fcat", "fcat") {

  // std::string fastcat_config_path;
  // this->declare_parameter<std::string>("fastcat_config_path", "");
  // this->get_parameter("fastcat_config_path", fastcat_config_path);

  // MSG("loading Yaml from %s", fastcat_config_path.c_str());
  // YAML::Node node = YAML::LoadFile(fastcat_config_path);

  // if (!fcat_manager_.ConfigFromYaml(node)) {
  //   throw std::invalid_argument(
  //       "Fastcat Manager failed to process bus configuration YAML file.");
  // }
  // MSG("Created Fcat Manager!");

  // this->declare_parameter<bool>("create_joint_state_pub", true);
  // this->get_parameter("create_joint_state_pub", enable_js_pub_);

  // PopulateDeviceStateFields();

  // // Must populate certain device state fields before initializing pub/sub
  // InitializePublishersAndMessages();
  // InitializeSubscribers();

  // SetTimerRate(fcat_manager_.GetTargetLoopRate());
  // last_time_ = fcat_get_time_sec();
  // MSG("Starting Wall Timer at %lf hz", fcat_manager_.GetTargetLoopRate());
  // InitializeTimer();

  process_loop_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  topic_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  std::string fastcat_config_path = DeclareInitParameterString(
      "fastcat_config_path",
      "",  // must be passed, cannot generally reason about this file path
      "Location of the the fastcat topology YAML file");

  MSG("loading Yaml from %s", fastcat_config_path.c_str());
  YAML::Node node = YAML::LoadFile(fastcat_config_path);

  if (!fcat_manager_.ConfigFromYaml(node)) {
    throw std::invalid_argument(
        "Fastcat Manager failed to process bus configuration YAML file.");
  }

  std::vector<std::string> gold_actuator_names;
  fcat_manager_.GetDeviceNamesByType(gold_actuator_names,
                                     fastcat::GOLD_ACTUATOR_STATE);

  std::vector<std::string> platinum_actuator_names;
  fcat_manager_.GetDeviceNamesByType(platinum_actuator_names,
                                     fastcat::PLATINUM_ACTUATOR_STATE);

  DeclareInitParameterInt("fastcat_actuator_count",
                          static_cast<int>(gold_actuator_names.size() +
                                           platinum_actuator_names.size()),
                          "Total number of fastcat actuators", 0, -1, true);

  int i = 0;
  InitializeActuatorParams(gold_actuator_names, i);
  InitializeActuatorParams(platinum_actuator_names, i);

  use_sim_time_  = this->get_parameter("use_sim_time").as_bool();
  enable_js_pub_ = DeclareInitParameterBool(
      "create_joint_state_pub", true,
      "If true, FCAT creates and publishes the \"/joint_states\" topic");

  enable_ros_wrench_pub_ = DeclareInitParameterBool(
      "create_ros_wrench_pub", true,
      "If true, FCAT creates and publishes individual topics for FTS devices");

  process_loop_cpu_id_ = DeclareInitParameterInt(
      "process_loop_cpu_id", 0,
      "Set the CPU ID for the main Process() thread; this settings should be "
      "used in conjunction with setting the kernel parameter 'isolcpus=0', so "
      "that the Process() thread is the only process running on CPU0; set to -1 "
      "to disable CPU pinning; it is expected that most applications will use "
      "either 0 or -1, but there may be exceptional cases where it is necessary "
      "to pin to a different CPU core; fcat will throw a fatal error if you attempt "
      "to pin to a CPU core value greater than the number of cores available "
      "(default: 0)",
      -1, 128);

  DeclareRuntimeParameterInt("csp_explicit_interpolation_cycles_delay", 3,
                             "Delays the onset of a motion profile in explicit "
                             "interpolation mode by a "
                             "number of cycles of the calling module; if set "
                             "to 3, then fcat will wait until "
                             "4 messages csp messages accumulate in the buffer "
                             "before initiating motion "
                             "profile.",
                             0, 10);

  DeclareRuntimeParameterInt(
      "csp_interpolation_cycles_stale", 10,
      "When fastcat is in CSP interpolation mode, module will "
      "transition to HOLDING state when it has not received a "
      "new CSP setpoint within the number of internal loop "
      "cycles configured by this parameter",
      4, 40);

  DeclareRuntimeParameterString(
      "csp_explicit_interpolation_algorithm", "cubic",
      "Specify the algorithm used for explicit interpolation; "
      "one of ['cubic', 'linear']: cubic interpolation is a third order "
      "interpolation "
      "of position and velocity with c2 continuity between csp set points; "
      "linear interpolation linearly interpolates both position and "
      "velocity between csp setpoints");

  DeclareRuntimeParameterString("csp_explicit_interpolation_timestamp_source",
                                "csp_message",
                                "Specify the source for the timestamp used for "
                                "csp interpolation in explicit mode; "
                                "If set to 'csp_message', fastcat uses the "
                                "request_time from the csp message, "
                                "if set to 'clock', fastcat uses the current "
                                "clock on receipt of the message; "
                                "['csp_message', 'clock']");

  DeclareRuntimeParameterBool("report_cycle_slips",
                              true,
                              "If true, fcat emits EVR messages to report timer "
                              "slips");
  
  PopulateDeviceStateFields();

  // Must populate certain device state fields before initializing pub/sub
  InitializePublishersAndMessages();
  InitializeSubscribers();
  InitializeServices();

  param_cb_handles_.push_back(this->add_on_set_parameters_callback(
      std::bind(&Fcat::SetParametersCb, this, _1)));

  // pull the Timer Rate from the Fastcat YAML
  InitializeTimerRate(fcat_manager_.GetTargetLoopRate());
  MSG("Starting Wall Timer at %lf hz",
                  fcat_manager_.GetTargetLoopRate());

  loop_period_sec_          = 1.0 / fcat_manager_.GetTargetLoopRate();
  last_time_                = this->now().seconds();
  module_state_msg_.faulted = false;

  Fcat::StartProcessTimer();
}

void Fcat::StartProcessTimer()
{
  rclcpp::Parameter rate_param;
  if (!this->get_parameter("target_loop_rate_hz", rate_param)) {
    MSG(
        "Developer Warning: must call InitializeTimerRate(...) before "
        "starting timer");
    rclcpp::shutdown();
  }

  double period_usec = 1.0e6 / fcat_manager_.GetTargetLoopRate();
  std::chrono::duration<double, std::micro> chrono_dur(period_usec);
  process_timer_ =
      rclcpp::create_timer(this, this->get_clock(), chrono_dur, 
                           std::bind(&FcatNode::Process, this),
                           process_loop_callback_group_);
}

rcl_interfaces::msg::SetParametersResult Fcat::SetParametersCb(
    const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "csp_explicit_interpolation_cycles_delay") {
      size_t delay = static_cast<size_t>(parameter.as_int());
      fcat_manager_.SetExplicitInterpolationCyclesDelay(delay);
      result.successful = true;
    } else if (parameter.get_name() == "csp_interpolation_cycles_stale") {
      size_t cycles = static_cast<size_t>(parameter.as_int());
      fcat_manager_.SetInterpolationCyclesStale(cycles);
      result.successful = true;
    } else if (parameter.get_name() == "csp_explicit_interpolation_algorithm") {
      auto value = parameter.as_string();
      if (value == "cubic") {
        fcat_manager_.SetExplicitInterpolationAlgorithmCubic();
        result.successful = true;
      } else if (value == "linear") {
        fcat_manager_.SetExplicitInterpolationAlgorithmLinear();
        result.successful = true;
      } else {
        result.reason =
            "Invalid algorithm requested, must be one of ['cubic', 'linear']";
        result.successful = false;
      }
    } else if (parameter.get_name() ==
               "csp_explicit_interpolation_timestamp_source") {
      auto value = parameter.as_string();
      if (value == "csp_message") {
        fcat_manager_.SetExplicitInterpolationTimestampSourceCspMessage();
        result.successful = true;
      } else if (value == "clock") {
        fcat_manager_.SetExplicitInterpolationTimestampSourceClock();
        result.successful = true;
      } else {
        result.reason =
            "Invalid algorithm requested, must be one of ['csp_message', "
            "'clock']";
        result.successful = false;
      }
    }
  }
  return result;
}

void Fcat::InitializeActuatorParams(
    const std::vector<std::string>& actuator_names, int& i)
{
  fastcat::Actuator::ActuatorParams actuator_params;
  for (auto& actuator_name : actuator_names) {
    if (fcat_manager_.GetActuatorParams(actuator_name, actuator_params)) {
      std::string parameter_prefix =
          "fastcat_actuator_" + std::to_string(++i) + "_";
      std::string description_prefix = "Actuator " + std::to_string(i) + " ";
      DeclareInitParameterString(parameter_prefix + "name", actuator_name,
                                 description_prefix + "name", true);
      DeclareInitParameterString(parameter_prefix + "actuator_type",
                                 actuator_params.actuator_type_str,
                                 description_prefix + "actuator type", true);
      DeclareInitParameterDouble(
          parameter_prefix + "gear_ratio", actuator_params.gear_ratio,
          description_prefix + "gear ratio", 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "counts_per_rev", actuator_params.counts_per_rev,
          description_prefix + "counts per revolution", 0.0, -1.0, true);
      DeclareInitParameterDouble(parameter_prefix + "max_speed_eu_per_sec",
                                 actuator_params.max_speed_eu_per_sec,
                                 description_prefix + "max speed [eu/sec]", 0.0,
                                 -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "max_accel_eu_per_sec2",
          actuator_params.max_accel_eu_per_sec2,
          description_prefix + "max acceleration [eu/sec^2]", 0.0, -1.0, true);
      DeclareInitParameterDouble(parameter_prefix + "over_speed_multiplier",
                                 actuator_params.over_speed_multiplier,
                                 description_prefix + "over speed multiplier",
                                 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "vel_tracking_error_eu_per_sec",
          actuator_params.vel_tracking_error_eu_per_sec,
          description_prefix + "velocity tracking error [eu/sec]", 0.0, -1.0,
          true);
      DeclareInitParameterDouble(
          parameter_prefix + "pos_tracking_error_eu",
          actuator_params.pos_tracking_error_eu,
          description_prefix + "position tracking error [eu]", 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "peak_current_limit_amps",
          actuator_params.peak_current_limit_amps,
          description_prefix + "peak current limit [amps]", 0.0, -1.0, true);
      DeclareInitParameterDouble(parameter_prefix + "peak_current_time_sec",
                                 actuator_params.peak_current_time_sec,
                                 description_prefix + "peak current time [sec]",
                                 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "continuous_current_limit_amps",
          actuator_params.continuous_current_limit_amps,
          description_prefix + "continuous current limit [amps]", 0.0, -1.0,
          true);
      DeclareInitParameterDouble(parameter_prefix + "torque_slope_amps_per_sec",
                                 actuator_params.torque_slope_amps_per_sec,
                                 description_prefix + "torque slope [amps/sec]",
                                 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "low_pos_cal_limit_eu",
          actuator_params.low_pos_cal_limit_eu,
          description_prefix + "low position calibration limit [eu]", 0.0, -1.0,
          true);
      DeclareInitParameterDouble(
          parameter_prefix + "low_pos_cmd_limit_eu",
          actuator_params.low_pos_cmd_limit_eu,
          description_prefix + "low position command limit [eu]", 0.0, -1.0,
          true);
      DeclareInitParameterDouble(
          parameter_prefix + "high_pos_cal_limit_eu",
          actuator_params.high_pos_cal_limit_eu,
          description_prefix + "high position calibration limit [eu]", 0.0,
          -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "high_pos_cmd_limit_eu",
          actuator_params.high_pos_cmd_limit_eu,
          description_prefix + "high position command limit [eu]", 0.0, -1.0,
          true);
      DeclareInitParameterDouble(parameter_prefix + "holding_duration_sec",
                                 actuator_params.holding_duration_sec,
                                 description_prefix + "holding duration [sec]",
                                 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "elmo_brake_engage_msec",
          actuator_params.elmo_brake_engage_msec,
          description_prefix + "brake engage time [msec]", 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "elmo_brake_disengage_msec",
          actuator_params.elmo_brake_disengage_msec,
          description_prefix + "brake disengage time [msec]", 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "elmo_crc", actuator_params.elmo_crc,
          description_prefix + "crc value", 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "elmo_drive_max_cur_limit_amps",
          actuator_params.elmo_drive_max_cur_limit_amps,
          description_prefix + "drive max current limit [amps]", 0.0, -1.0,
          true);
      DeclareInitParameterDouble(
          parameter_prefix + "smooth_factor", actuator_params.smooth_factor,
          description_prefix + "smoothing factor", 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "torque_constant", actuator_params.torque_constant,
          description_prefix + "torque constant", 0.0, -1.0, true);
      DeclareInitParameterDouble(parameter_prefix + "winding_resistance",
                                 actuator_params.winding_resistance,
                                 description_prefix + "winding resistance", 0.0,
                                 -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "brake_power", actuator_params.brake_power,
          description_prefix + "brake power", 0.0, -1.0, true);
      DeclareInitParameterDouble(
          parameter_prefix + "motor_encoder_gear_ratio",
          actuator_params.motor_encoder_gear_ratio,
          description_prefix + "motor encoder gear ratio", 0.0, -1.0, true);
      DeclareInitParameterBool(parameter_prefix + "has_absolute_encoder",
                               actuator_params.actuator_absolute_encoder,
                               description_prefix + "has absolute encoder",
                               true);
    }
  }
}

void Fcat::PopulateDeviceStateFields(){

  device_state_ptrs_ = fcat_manager_.GetDeviceStatePointers();

  for(auto it = device_state_ptrs_.begin(); it != device_state_ptrs_.end(); ++it){
    MSG("Populating device_name_state_map_[%s]", (*it)->name.c_str());

    device_name_state_map_[(*it)->name] = *it;

    device_type_vec_map_[(*it)->type].push_back(*it);

    MSG("counts[%d] = %ld", (*it)->type, device_type_vec_map_[(*it)->type].size());
  }
}

bool Fcat::ActuatorExistsOnBus(const std::string& name)
{
  if (DeviceExistsOnBus(name, fastcat::GOLD_ACTUATOR_STATE)) {
    return true;
  } else if (DeviceExistsOnBus(name, fastcat::PLATINUM_ACTUATOR_STATE)) {
    return true;
  }
  // handle other future Actuator device types here
  return false;
}

bool Fcat::ActuatorExistsOnBus(const std::string& name,
                               std::string&       error_message)
{
  if (DeviceExistsOnBus(name, fastcat::GOLD_ACTUATOR_STATE, error_message)) {
    return true;
  } else if (DeviceExistsOnBus(name, fastcat::PLATINUM_ACTUATOR_STATE,
                               error_message)) {
    return true;
  }
  // handle other future Actuator device types here
  return false;
}

bool Fcat::DeviceExistsOnBus(std::string name, fastcat::DeviceStateType type)
{
  if(!(device_name_state_map_.end() != device_name_state_map_.find(name))){
    //ERROR("Device %s does not exist on bus", name.c_str());
    snprintf(str, 512, "Device %s does not exist on bus", name.c_str());
    error_message = str;
    return false;
  }
  if(device_name_state_map_[name]->type != type){
    //ERROR("Device type does not match for %s", name.c_str());
    snprintf(str, 512, "Device type does not match for %s", name.c_str());
    error_message = str;
    return false;
  }
  return true;
}

bool Fcat::DeviceExistsOnBus(const std::string&       name,
                             fastcat::DeviceStateType type)
{
  std::string error_message;
  bool        success = DeviceExistsOnBus(name, type, error_message);
  if (not success) {
    MSG("%s", error_message.c_str());
  }
  return success;
}

bool Fcat::TypeExistsOnBus(fastcat::DeviceStateType type)
{
  if (device_type_vec_map_[type].size() > 0) {
    return true;
  }
  return false;
}

void Fcat::InitializePublishersAndMessages(){

  MSG("Fcat::InitializePublishers()");

  // Async SDO Response
  async_sdo_response_pub_ =
      this->create_publisher<fcat_msgs::msg::AsyncSdoResponse>(
          "state/async_sdo_response", publisher_queue_size_);

  // Actuator
  //auto vec_state_ptrs = device_type_vec_map_[fastcat::ACTUATOR_STATE];
  auto gold_vec_state_ptrs = device_type_vec_map_[fastcat::GOLD_ACTUATOR_STATE];
  auto platinum_vec_state_ptrs =
      device_type_vec_map_[fastcat::PLATINUM_ACTUATOR_STATE];
  size_t num_actuators =
      gold_vec_state_ptrs.size() + platinum_vec_state_ptrs.size();
  if(vec_state_ptrs.size() > 0){

    MSG("Creating actuator pub");
    actuator_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorStates>(
        "state/actuators", publisher_queue_size_);

    actuator_states_msg_.names.resize(vec_state_ptrs.size());
    actuator_states_msg_.states.resize(vec_state_ptrs.size());

    if(enable_js_pub_){
      MSG("Creating joint_states pub");

      // This is the only topic that broadcasts in the global 
      // namespace. This prevents needing to remap `fcat/state/joint_states` 
      // to `/joint_states` in launch files or through command line args.
      // It can always be reverted if this causes any issues down the road.
      joint_state_pub_ = 
        this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", publisher_queue_size_);
    }
  }

  // Egd
  vec_state_ptrs = device_type_vec_map_[fastcat::EGD_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Egd pub");
    egd_pub_ = this->create_publisher<fcat_msgs::msg::EgdStates>(
        "state/egds", publisher_queue_size_);

    egd_states_msg_.names.resize(vec_state_ptrs.size());
    egd_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El1008
  vec_state_ptrs = device_type_vec_map_[fastcat::EL1008_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating El1008 pub");
    el1008_pub_ = this->create_publisher<fcat_msgs::msg::El1008States>(
        "state/el1008s", publisher_queue_size_);

    el1008_states_msg_.names.resize(vec_state_ptrs.size());
    el1008_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El2124
  vec_state_ptrs = device_type_vec_map_[fastcat::EL2124_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating El2124 pub");
    el2124_pub_ = this->create_publisher<fcat_msgs::msg::El2124States>(
        "state/el2124s", publisher_queue_size_);

    el2124_states_msg_.names.resize(vec_state_ptrs.size());
    el2124_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El2809
  vec_state_ptrs = device_type_vec_map_[fastcat::EL2809_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating El2809 pub");
    el2809_pub_ = this->create_publisher<fcat_msgs::msg::El2809States>(
        "state/el2809s", publisher_queue_size_);

    el2809_states_msg_.names.resize(vec_state_ptrs.size());
    el2809_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3104
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3104_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating El3104 pub");
    el3104_pub_ = this->create_publisher<fcat_msgs::msg::El3104States>(
        "state/el3104s", publisher_queue_size_);

    el3104_states_msg_.names.resize(vec_state_ptrs.size());
    el3104_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3162
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3162_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating El3162 pub");
    el3162_pub_ = this->create_publisher<fcat_msgs::msg::El3162States>(
        "state/el3162s", publisher_queue_size_);

    el3162_states_msg_.names.resize(vec_state_ptrs.size());
    el3162_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3202
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3202_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating El3202 pub");
    el3202_pub_ = this->create_publisher<fcat_msgs::msg::El3202States>(
        "state/el3202s", publisher_queue_size_);

    el3202_states_msg_.names.resize(vec_state_ptrs.size());
    el3202_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3208
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3208_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating El3208 pub");
    el3208_pub_ = this->create_publisher<fcat_msgs::msg::El3208States>(
        "state/el3208s", publisher_queue_size_);

    el3208_states_msg_.names.resize(vec_state_ptrs.size());
    el3208_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3318
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3318_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating El3318 pub");
    el3318_pub_ = this->create_publisher<fcat_msgs::msg::El3318States>(
        "state/el3318s", publisher_queue_size_);

    el3318_states_msg_.names.resize(vec_state_ptrs.size());
    el3318_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3602
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3602_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating El3602 pub");
    el3602_pub_ = this->create_publisher<fcat_msgs::msg::El3602States>(
        "state/el3602s", publisher_queue_size_);

    el3602_states_msg_.names.resize(vec_state_ptrs.size());
    el3602_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El4102
  vec_state_ptrs = device_type_vec_map_[fastcat::EL4102_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating El4102 pub");
    el4102_pub_ = this->create_publisher<fcat_msgs::msg::El4102States>(
        "state/el4102s", publisher_queue_size_);

    el4102_states_msg_.names.resize(vec_state_ptrs.size());
    el4102_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // // Jed
  // vec_state_ptrs = device_type_vec_map_[fastcat::JED_STATE];
  // if(vec_state_ptrs.size() > 0){

  //   MSG("Creating Jed pub");
  //   jed_pub_ = this->create_publisher<fcat_msgs::msg::JedStates>(
  //       "state/jeds", publisher_queue_size_);

  //   jed_states_msg_.names.resize(vec_state_ptrs.size());
  //   jed_states_msg_.states.resize(vec_state_ptrs.size());
  // }

  // ILD1900
  vec_state_ptrs = device_type_vec_map_[fastcat::ILD1900_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating Ild1900 pub");
    ild1900_pub_ = this->create_publisher<fcat_msgs::msg::Ild1900States>(
        "state/ild1900s", publisher_queue_size_);

    ild1900_states_msg_.names.resize(vec_state_ptrs.size());
    ild1900_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Commander
  vec_state_ptrs = device_type_vec_map_[fastcat::COMMANDER_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Commander pub");
    commander_pub_ = this->create_publisher<fcat_msgs::msg::CommanderStates>(
        "state/commanders", publisher_queue_size_);

    commander_states_msg_.names.resize(vec_state_ptrs.size());
    commander_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Conditional
  vec_state_ptrs = device_type_vec_map_[fastcat::CONDITIONAL_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Conditional pub");
    conditional_pub_ = this->create_publisher<fcat_msgs::msg::ConditionalStates>(
        "state/conditionals", publisher_queue_size_);

    conditional_states_msg_.names.resize(vec_state_ptrs.size());
    conditional_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Faulter
  vec_state_ptrs = device_type_vec_map_[fastcat::FAULTER_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Faulter pub");
    faulter_pub_ = this->create_publisher<fcat_msgs::msg::FaulterStates>(
        "state/faulters", publisher_queue_size_);

    faulter_states_msg_.names.resize(vec_state_ptrs.size());
    faulter_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Filter
  vec_state_ptrs = device_type_vec_map_[fastcat::FILTER_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Filter pub");
    filter_pub_ = this->create_publisher<fcat_msgs::msg::FilterStates>(
        "state/filter", publisher_queue_size_);

    filter_states_msg_.names.resize(vec_state_ptrs.size());
    filter_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Function
  vec_state_ptrs = device_type_vec_map_[fastcat::FUNCTION_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Function pub");
    function_pub_ = this->create_publisher<fcat_msgs::msg::FunctionStates>(
        "state/functions", publisher_queue_size_);

    function_states_msg_.names.resize(vec_state_ptrs.size());
    function_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Pid
  vec_state_ptrs = device_type_vec_map_[fastcat::PID_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Pid pub");
    pid_pub_ = this->create_publisher<fcat_msgs::msg::PidStates>(
        "state/pids", publisher_queue_size_);

    pid_states_msg_.names.resize(vec_state_ptrs.size());
    pid_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Saturation
  vec_state_ptrs = device_type_vec_map_[fastcat::SATURATION_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Saturation pub");
    saturation_pub_ = this->create_publisher<fcat_msgs::msg::SaturationStates>(
        "state/saturations", publisher_queue_size_);

    saturation_states_msg_.names.resize(vec_state_ptrs.size());
    saturation_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Schmitt Trigger
  vec_state_ptrs = device_type_vec_map_[fastcat::SCHMITT_TRIGGER_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Schmitt Trigger pub");
    schmitt_trigger_pub_ = this->create_publisher<fcat_msgs::msg::SchmittTriggerStates>(
        "state/schmitt_triggers", publisher_queue_size_);

    schmitt_trigger_states_msg_.names.resize(vec_state_ptrs.size());
    schmitt_trigger_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Signal Generator
  vec_state_ptrs = device_type_vec_map_[fastcat::SIGNAL_GENERATOR_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Signal Generator pub");
    signal_generator_pub_ = this->create_publisher<fcat_msgs::msg::SignalGeneratorStates>(
        "state/signal_generators", publisher_queue_size_);

    signal_generator_states_msg_.names.resize(vec_state_ptrs.size());

    signal_generator_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Linear Interpolation
  vec_state_ptrs = device_type_vec_map_[fastcat::LINEAR_INTERPOLATION_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating LinearInterpolation pub");
    linear_interpolation_pub_ =
        this->create_publisher<fcat_msgs::msg::LinearInterpolationStates>(
            "state/linear_interpolators", publisher_queue_size_);

    linear_interpolation_states_msg_.names.resize(vec_state_ptrs.size());
    linear_interpolation_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Three Node Thermal Model
  vec_state_ptrs =
      device_type_vec_map_[fastcat::THREE_NODE_THERMAL_MODEL_STATE];
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating ThreeNodeThermalModel pub");
    three_node_thermal_model_pub_ =
        this->create_publisher<fcat_msgs::msg::ThreeNodeThermalModelStates>(
            "state/three_node_thermal_models", publisher_queue_size_);

    three_node_thermal_model_states_msg_.names.resize(vec_state_ptrs.size());
    three_node_thermal_model_states_msg_.states.resize(vec_state_ptrs.size());
  }
  
  // FTS, AtiFts, VirtualFts
  vec_state_ptrs = device_type_vec_map_[fastcat::FTS_STATE];
  // for(auto dev_state_ptr = vec_state_ptrs.begin(); dev_state_ptr != vec_state_ptrs.end(); dev_state_ptr++){

  //    std::string device = (*dev_state_ptr)->name;
  //    fts_raw_pub_map_[device] = 
  //      this->create_publisher<geometry_msgs::msg::Wrench>(
  //        "fts/" + device + "/raw_wrench", publisher_queue_size_);

  //    fts_tared_pub_map_[device] = 
  //      this->create_publisher<geometry_msgs::msg::Wrench>(
  //        "fts/" + device + "/tared_wrench", publisher_queue_size_);
  //  }
  if (vec_state_ptrs.size() > 0) {
    MSG("Creating FTS pub");

    fts_pub_ = this->create_publisher<fcat_msgs::msg::FtsStates>(
        "state/fts", publisher_queue_size_);

    fts_states_msg_.names.resize(vec_state_ptrs.size());
    fts_states_msg_.states.resize(vec_state_ptrs.size());

    if (enable_ros_wrench_pub_) {
      for (auto dev_state_ptr = vec_state_ptrs.begin();
           dev_state_ptr != vec_state_ptrs.end(); dev_state_ptr++) {
        std::string device = (*dev_state_ptr)->name;
        fts_raw_pub_map_[device] =
            this->create_publisher<geometry_msgs::msg::WrenchStamped>(
                "fts/" + device + "/raw_wrench", publisher_queue_size_);

        fts_tared_pub_map_[device] =
            this->create_publisher<geometry_msgs::msg::WrenchStamped>(
                "fts/" + device + "/tared_wrench", publisher_queue_size_);

        MSG("Creating ROS WrenchStamped Topics for %s",
                        device.c_str());
      }
    }
  }

  module_state_pub_ = this->create_publisher<fcat_msgs::msg::ModuleState>(
      "state/module_state", publisher_queue_size_);
}


void Fcat::InitializeSubscribers(){

  // bus reset/fault
  // reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
  //     "cmd/reset",
  //     subscriber_queue_size_,
  //     std::bind(&Fcat::ResetCb, this, _1));

  // fault_sub_ = this->create_subscription<std_msgs::msg::Empty>(
  //     "cmd/fault",
  //     subscriber_queue_size_,
  //     std::bind(&Fcat::FaultCb, this, _1));
  subscriptions_.push_back(this->create_subscription<std_msgs::msg::Empty>(
      "impl/reset", subscription_queue_size_,
      std::bind(&Fcat::ResetCb, this, _1), options));

  subscriptions_.push_back(this->create_subscription<std_msgs::msg::Empty>(
      "impl/fault", subscription_queue_size_,
      std::bind(&Fcat::FaultCb, this, _1), options));

  // Async SDO

  subscriptions_.push_back(
      this->create_subscription<fcat_msgs::msg::AsyncSdoReadCmd>(
          "impl/async_sdo_read", subscription_queue_size_,
          std::bind(&Fcat::AsyncSdoReadCmdCb, this, _1), options));

  subscriptions_.push_back(
      this->create_subscription<fcat_msgs::msg::AsyncSdoWriteCmd>(
          "impl/async_sdo_write", subscription_queue_size_,
          std::bind(&Fcat::AsyncSdoWriteCmdCb, this, _1), options));

  // Actuator
  // if(TypeExistsOnBus(fastcat::ACTUATOR_STATE)){

  //   actuator_csp_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorCspCmd>(
  //       "cmd/actuator_csp", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::ActuatorCSPCmdCb, this, _1));

  //   actuator_csv_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorCsvCmd>(
  //       "cmd/actuator_csv", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::ActuatorCSVCmdCb, this, _1));

  //   actuator_cst_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorCstCmd>(
  //       "cmd/actuator_cst", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::ActuatorCSTCmdCb, this, _1));

  //   actuator_calibrate_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorCalibrateCmd>(
  //       "cmd/actuator_calibrate", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::ActuatorCalibrateCmdCb, this, _1));

  //   actuator_prof_pos_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorProfPosCmd>(
  //       "cmd/actuator_prof_pos", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::ActuatorProfPosCmdCb, this, _1));

  //   actuator_prof_torque_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorProfTorqueCmd>(
  //       "cmd/actuator_prof_torque", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::ActuatorProfTorqueCmdCb, this, _1));

  //   actuator_prof_vel_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorProfVelCmd>(
  //       "cmd/actuator_prof_vel", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::ActuatorProfVelCmdCb, this, _1));

  //   actuator_set_output_position_sub_ = 
  //     this->create_subscription<fcat_msgs::msg::ActuatorSetOutputPositionCmd>(
  //         "cmd/actuator_set_output_position", 
  //         subscriber_queue_size_, 
  //         std::bind(&Fcat::ActuatorSetOutputPositionCmdCb, this, _1));
  //}
    if (TypeExistsOnBus(fastcat::GOLD_ACTUATOR_STATE) ||
      TypeExistsOnBus(fastcat::PLATINUM_ACTUATOR_STATE)) {
    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorCspCmd>(
            "impl/actuator_csp", subscription_queue_size_,
            std::bind(&Fcat::ActuatorCSPCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorCsvCmd>(
            "impl/actuator_csv", subscription_queue_size_,
            std::bind(&Fcat::ActuatorCSVCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorCstCmd>(
            "impl/actuator_cst", subscription_queue_size_,
            std::bind(&Fcat::ActuatorCSTCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorCspCmds>(
            "impl/actuator_csp_multi", subscription_queue_size_,
            std::bind(&Fcat::ActuatorCSPCmdsCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorCalibrateCmd>(
            "impl/actuator_calibrate", subscription_queue_size_,
            std::bind(&Fcat::ActuatorCalibrateCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorProfPosCmd>(
            "impl/actuator_prof_pos", subscription_queue_size_,
            std::bind(&Fcat::ActuatorProfPosCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorProfTorqueCmd>(
            "impl/actuator_prof_torque", subscription_queue_size_,
            std::bind(&Fcat::ActuatorProfTorqueCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorProfVelCmd>(
            "impl/actuator_prof_vel", subscription_queue_size_,
            std::bind(&Fcat::ActuatorProfVelCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorProfPosCmds>(
            "impl/actuator_prof_pos_multi", subscription_queue_size_,
            std::bind(&Fcat::ActuatorProfPosCmdsCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorSetOutputPositionCmd>(
            "impl/actuator_set_output_position", subscription_queue_size_,
            std::bind(&Fcat::ActuatorSetOutputPositionCmdCb, this, _1),
            options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorSetDigitalOutputCmd>(
            "impl/actuator_set_digital_output", subscription_queue_size_,
            std::bind(&Fcat::ActuatorSetDigitalOutputCmdCb, this, _1),
            options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorSetMaxCurrentCmd>(
            "impl/actuator_set_max_current", subscription_queue_size_,
            std::bind(&Fcat::ActuatorSetMaxCurrentCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorSetUnitModeCmd>(
            "impl/actuator_set_unit_mode", subscription_queue_size_,
            std::bind(&Fcat::ActuatorSetUnitModeCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<
            fcat_msgs::msg::ActuatorSetProfDisengagingTimeoutCmd>(
            "impl/actuator_set_prof_disengaging_timeout",
            subscription_queue_size_,
            std::bind(&Fcat::ActuatorSetProfDisengagingTimeoutCmdCb, this, _1),
            options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorHaltCmd>(
            "impl/actuator_halt", subscription_queue_size_,
            std::bind(&Fcat::ActuatorHaltCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorHaltCmds>(
            "impl/actuator_halt_multi", subscription_queue_size_,
            std::bind(&Fcat::ActuatorHaltCmdsCb, this, _1), options));
  }

  // Commander
  if (TypeExistsOnBus(fastcat::COMMANDER_STATE)) {
    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::CommanderEnableCmd>(
            "impl/commander_enable", subscription_queue_size_,
            std::bind(&Fcat::CommanderEnableCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::CommanderDisableCmd>(
            "impl/commander_disable", subscription_queue_size_,
            std::bind(&Fcat::CommanderDisableCmdCb, this, _1), options));
  }

  // // El2124
  // if(TypeExistsOnBus(fastcat::EL2124_STATE)){
  //   el2124_write_all_channels_sub_ = 
  //     this->create_subscription<fcat_msgs::msg::El2124WriteAllChannelsCmd>(
  //         "cmd/el2124_write_all_channels", 
  //         subscriber_queue_size_, 
  //         std::bind(&Fcat::El2124WriteAllChannelsCmdCb, this, _1));

  //   el2124_write_channel_sub_ = 
  //     this->create_subscription<fcat_msgs::msg::El2124WriteChannelCmd>(
  //         "cmd/el2124_write_channel", 
  //         subscriber_queue_size_, 
  //         std::bind(&Fcat::El2124WriteChannelCmdCb, this, _1));
  // }
  // El2124
  if (TypeExistsOnBus(fastcat::EL2124_STATE)) {
    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::El2124WriteAllChannelsCmd>(
            "impl/el2124_write_all_channels", subscription_queue_size_,
            std::bind(&Fcat::El2124WriteAllChannelsCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::El2124WriteChannelCmd>(
            "impl/el2124_write_channel", subscription_queue_size_,
            std::bind(&Fcat::El2124WriteChannelCmdCb, this, _1), options));
  }

  // El2809
  if (TypeExistsOnBus(fastcat::EL2809_STATE)) {
    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::El2809WriteAllChannelsCmd>(
            "impl/el2809_write_all_channels", subscription_queue_size_,
            std::bind(&Fcat::El2809WriteAllChannelsCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::El2809WriteChannelCmd>(
            "impl/el2809_write_channel", subscription_queue_size_,
            std::bind(&Fcat::El2809WriteChannelCmdCb, this, _1), options));
  }

  // EL4102
  if (TypeExistsOnBus(fastcat::EL4102_STATE)) {
    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::El4102WriteAllChannelsCmd>(
            "impl/el4102_write_all_channels", subscription_queue_size_,
            std::bind(&Fcat::El4102WriteAllChannelsCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::El4102WriteChannelCmd>(
            "impl/el4102_write_channel", subscription_queue_size_,
            std::bind(&Fcat::El4102WriteChannelCmdCb, this, _1), options));
  }

  // // Faulter
  // if(TypeExistsOnBus(fastcat::FAULTER_STATE)){
  //   faulter_enable_sub_ = this->create_subscription<fcat_msgs::msg::FaulterEnableCmd>(
  //       "cmd/faulter_enable", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::FaulterEnableCmdCb, this, _1));
  // }

  // // Fts
  // if(TypeExistsOnBus(fastcat::FTS_STATE)){
  //   fts_tare_sub_= this->create_subscription<fcat_msgs::msg::FtsTareCmd>(
  //       "cmd/fts_tare", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::FtsTareCmdCb, this, _1));
  // }

  // Faulter
  if (TypeExistsOnBus(fastcat::FAULTER_STATE)) {
    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::FaulterEnableCmd>(
            "impl/faulter_enable", subscription_queue_size_,
            std::bind(&Fcat::FaulterEnableCmdCb, this, _1), options));
  }

  // Fts
  if (TypeExistsOnBus(fastcat::FTS_STATE)) {
    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::FtsTareCmd>(
            "impl/fts_tare", subscription_queue_size_,
            std::bind(&Fcat::FtsTareCmdCb, this, _1), options));
  }

  // // Jed
  // if(TypeExistsOnBus(fastcat::JED_STATE)){
  //   jed_set_cmd_value_sub_= this->create_subscription<fcat_msgs::msg::JedSetCmdValueCmd>(
  //       "cmd/jed_set_cmd_value", 
  //       subscriber_queue_size_, 
  //       std::bind(&Fcat::JedSetCmdValueCmdCb, this, _1));
  // }

//   // Pid
//   if(TypeExistsOnBus(fastcat::PID_STATE)){
//     pid_activate_sub_= this->create_subscription<fcat_msgs::msg::PidActivateCmd>(
//         "cmd/pid_activate", 
//         subscriber_queue_size_, 
//         std::bind(&Fcat::PidActivateCmdCb, this, _1));
//   }
  if (TypeExistsOnBus(fastcat::PID_STATE)) {
    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::PidActivateCmd>(
            "impl/pid_activate", subscription_queue_size_,
            std::bind(&Fcat::PidActivateCmdCb, this, _1), options));
  }
}

void Fcat::InitializeServices()
{
  // bus reset/fault
  this->create_service<std_srvs::srv::Trigger>(
      this, "cmd/reset", &Fcat::ResetSrvCb,
      );

  this->create_service<std_srvs::srv::Trigger>(
      this, "cmd/fault", &Fcat::FaultSrvCb
      );

  // Actuator
  if (TypeExistsOnBus(fastcat::GOLD_ACTUATOR_STATE) ||
      TypeExistsOnBus(fastcat::PLATINUM_ACTUATOR_STATE)) {
    this->create_service<fcat_msgs::srv::ActuatorHaltService>(
        this, "cmd/actuator_halt", &Fcat::ActuatorHaltSrvCb
        );

    this->create_service<
        fcat_msgs::srv::ActuatorSetGainSchedulingIndexService>(
        this, "cmd/actuator_set_gain_scheduling_index",
        &Fcat::ActuatorSetGainSchedulingIndexSrvCb
        );

    this->create_service<fcat_msgs::srv::ActuatorSetMaxCurrentService>(
        this, "cmd/actuator_set_max_current", &Fcat::ActuatorSetMaxCurrentSrvCb
        );

    this->create_service<fcat_msgs::srv::ActuatorSetOutputPositionService>(
        this, "cmd/actuator_set_output_position",
        &Fcat::ActuatorSetOutputPositionSrvCb
        );

    this->create_service<fcat_msgs::srv::ActuatorSetDigitalOutputService>(
        this, "cmd/actuator_set_digital_output",
        &Fcat::ActuatorSetDigitalOutputSrvCb
        );

    this->create_service<
        fcat_msgs::srv::ActuatorSetProfDisengagingTimeoutService>(
        this, "cmd/actuator_set_prof_disengaging_timeout",
        &Fcat::ActuatorSetProfDisengagingTimeoutSrvCb
        );

    this->create_service<fcat_msgs::srv::ActuatorProfPosService>(
        this, "cmd/actuator_prof_pos", &Fcat::ActuatorProfPosSrvCb
        );

    this->create_service<fcat_msgs::srv::ActuatorProfVelService>(
        this, "cmd/actuator_prof_vel", &Fcat::ActuatorProfVelSrvCb
        );

    this->create_service<fcat_msgs::srv::ActuatorProfTorqueService>(
        this, "cmd/actuator_prof_torque", &Fcat::ActuatorProfTorqueSrvCb
        );

    this->create_service<fcat_msgs::srv::ActuatorCalibrateService>(
        this, "cmd/actuator_calibrate", &Fcat::ActuatorCalibrateSrvCb
        );

  }  // end Actuator Service Declarations

  // Commander
  if (TypeExistsOnBus(fastcat::COMMANDER_STATE)) {
    this->create_service<fcat_msgs::srv::CommanderEnableService>(
        this, "cmd/commander_enable", &Fcat::CommanderEnableSrvCb
        );

    this->create_service<fcat_msgs::srv::CommanderDisableService>(
        this, "cmd/commander_disable", &Fcat::CommanderDisableSrvCb
        );
  }  // end Commander Service Declarations

  // El2124
  if (TypeExistsOnBus(fastcat::EL2124_STATE)) {
    this->create_service<fcat_msgs::srv::El2124WriteAllChannelsService>(
        this, "cmd/el2124_write_all_channels",
        &Fcat::El2124WriteAllChannelsSrvCb
        );

    this->create_service<fcat_msgs::srv::El2124WriteChannelService>(
        this, "cmd/el2124_write_channel", &Fcat::El2124WriteChannelSrvCb
        );

  }  // end El2124 Service Declarations

  // El2809
  if (TypeExistsOnBus(fastcat::EL2809_STATE)) {
    this->create_service<fcat_msgs::srv::El2809WriteAllChannelsService>(
        this, "cmd/el2809_write_all_channels",
        &Fcat::El2809WriteAllChannelsSrvCb
        );

    this->create_service<fcat_msgs::srv::El2809WriteChannelService>(
        this, "cmd/el2809_write_channel", &Fcat::El2809WriteChannelSrvCb
        );

  }  // end El2809 Service Declarations

  // El4102
  if (TypeExistsOnBus(fastcat::EL4102_STATE)) {
    this->create_service<fcat_msgs::srv::El4102WriteAllChannelsService>(
        this, "cmd/el4102_write_all_channels",
        &Fcat::El4102WriteAllChannelsSrvCb
    );

    this->create_service<fcat_msgs::srv::El4102WriteChannelService>(
        this, "cmd/el4102_write_channel", &Fcat::El4102WriteChannelSrvCb
    );

  }  // end El4102 Service Declarations

  // Faulter
  if (TypeExistsOnBus(fastcat::FAULTER_STATE)) {
    this->create_service<fcat_msgs::srv::FaulterEnableService>(
        this, "cmd/faulter_enable", &Fcat::FaulterEnableSrvCb
    );

  }  // end Faulter Service Declarations

  // FTS
  if (TypeExistsOnBus(fastcat::FTS_STATE)) {
    this->create_service<fcat_msgs::srv::DeviceTriggerService>(
        this, "cmd/fts_tare", &Fcat::FtsTareSrvCb
    );

  }  // end FTS Service Declarations

  // PID
  if (TypeExistsOnBus(fastcat::PID_STATE)) {
    rclcpp::Service<fcat_msgs::srv::PidActivateService>::SharedPtr service = 
    this->create_service<fcat_msgs::srv::PidActivateService>(
        "cmd/pid_activate", &Fcat::PidActivateSrvCb
        );

  }  // end PID Service Declarations
}

void Fcat::Process(){

  // fcat_manager_.Process();

  // PublishModuleState();

  // PublishFtsStates();

  // PublishActuatorStates();
  // PublishEgdStates();
  // PublishEl2124States();
  // PublishEl3208States();
  // PublishEl3602States();
  // PublishJedStates();

  // PublishCommanderStates();
  // PublishConditionalStates();
  // PublishFaulterStates();
  // PublishFilterStates();
  // PublishFunctionStates();
  // PublishPidStates();
  // PublishSaturationStates();
  // PublishSchmittTriggerStates();
  // PublishSignalGeneratorStates();

    auto now = this->get_clock()->now();
  
  bool report_cycle_slips = this->get_parameter("report_cycle_slips").as_bool();
  if (report_cycle_slips && time_stamp_initialized_ && cpu_affinity_initialized_) {
    double dt = now.seconds() - publish_time_stamp_.seconds();
    if (dt > 2.0 * loop_period_sec_) {
      MSG(
          "Cycle slip detected; seconds since last Process() call: "
          "%f",
          dt);
    }
  }

  publish_time_stamp_     = now;
  time_stamp_initialized_ = true;

#ifdef _GNU_SOURCE
  // sched_affinity is a GNU extension, and may not be available in some
  // compilers
  if (!cpu_affinity_initialized_) {
    if (process_loop_cpu_id_ < 0) {
      // no thread pinning requested
      cpu_affinity_initialized_ = true;
    } else if (gettid() != getpid()) {

      int cpus_available = get_nprocs_conf();
      MSG("This system has %d CPUs configured", cpus_available);
      if(process_loop_cpu_id_ >= cpus_available) {
        MSG(
          "User requested to pin Process() function to CPU ID %d, but "
          "only %d CPUs are available on this machine", 
          process_loop_cpu_id_, cpus_available);
        rclcpp::shutdown();
      } else {
        cpu_set_t affinity;
        CPU_ZERO(&affinity);
        CPU_SET(process_loop_cpu_id_, &affinity);

        MSG("Process loop started in thread ID: %d", gettid());
        int result = sched_setaffinity(gettid(), sizeof(cpu_set_t), &affinity);
        if (result != 0) {
          MSG(
              "Coult not set process affinity to CPU %d for thread ID: %d, "
              "sched_setaffinity returned result: %d",
              process_loop_cpu_id_, gettid(), result);
        } else {
          MSG("Set CPU affinity to CPU %d for thread ID: %d",
                         process_loop_cpu_id_, gettid());
        }
        process_loop_thread_id_   = gettid();
        cpu_affinity_initialized_ = true;
      }
    }
  }
#else
  MSG(
    "GNU extensions are not available on this machine/compiler; the Process() loop "
    "cannot be moved to CPU %d", process_loop_thread_id_);
  cpu_affinity_initialized_ = true;
#endif

  if (use_sim_time_) {
    fcat_manager_.Process(publish_time_stamp_.seconds());
  } else {
    fcat_manager_.Process();
  }

  PublishAsyncSdoResponse();
  PublishModuleState();
  PublishFtsStates();
  PublishActuatorStates();
  PublishEgdStates();
  PublishEl1008States();
  PublishEl2124States();
  PublishEl2809States();
  PublishEl3104States();
  PublishEl3162States();
  PublishEl3202States();
  PublishEl3208States();
  PublishEl3318States();
  PublishEl3602States();
  PublishEl4102States();
  PublishIld1900States();

  PublishCommanderStates();
  PublishConditionalStates();
  PublishFaulterStates();
  PublishFilterStates();
  PublishFunctionStates();
  PublishPidStates();
  PublishSaturationStates();
  PublishSchmittTriggerStates();
  PublishSignalGeneratorStates();
  PublishLinearInterpolationStates();
  PublishThreeNodeThermalModelStates();
}

void Fcat::PublishModuleState(){
  // auto module_state_msg = fcat_msgs::msg::ModuleState();

  // module_state_msg.faulted = fcat_manager_.IsFaulted();

  // double now = fcat_get_time_sec();
  // module_state_msg.jitter = now - last_time_;
  // last_time_ = now;

  // Check for fault statu and emit EVRs
  bool is_faulted = fcat_manager_.IsFaulted();

  if (!module_state_msg_.faulted && is_faulted) {
    MSG("Fastcat bus fault detected");
  } else if (module_state_msg_.faulted && !is_faulted) {
    MSG("Fastcat bus fault cleared");
  }

  module_state_msg_.faulted            = is_faulted;
  module_state_msg_.command_queue_size = command_queue_size_;
  command_queue_size_                  = 0;

  // Populate the rest of the message
  double t = this->now().seconds();

  // jitter is defined as the delta between the last time the module
  // was serviced and the nominal loop period. e.g. if a module
  // is serviced exactly at its nominal loop period, it will have zero jitter
  module_state_msg_.jitter = t - last_time_ - loop_period_sec_;
  last_time_               = t;

  module_state_msg_.header.stamp = publish_time_stamp_;

  module_state_pub_->publish(module_state_msg_);
}

void Fcat::PublishAsyncSdoResponse()
{
  fastcat::SdoResponse sdo_resp;
  if (fcat_manager_.PopSdoResponseQueue(sdo_resp)) {
    fcat_msgs::msg::AsyncSdoResponse msg;

    msg.device_name = sdo_resp.device_name;
    msg.request_type =
        jsd_sdo_request_type_to_string(sdo_resp.response.request_type);
    msg.sdo_index    = sdo_resp.response.sdo_index;
    msg.sdo_subindex = sdo_resp.response.sdo_subindex;
    msg.data_type    = jsd_sdo_data_type_to_string(sdo_resp.response.data_type);
    msg.app_id       = sdo_resp.response.app_id;
    msg.success      = sdo_resp.response.success;

    msg.data = jsd_sdo_data_to_string(sdo_resp.response.data_type,
                                      sdo_resp.response.data);

    async_sdo_response_pub_->publish(msg);
  }
}

void Fcat::PublishFtsStates(){

  // FTS, AtiFts, VirtualFts
  auto vec_state_ptrs = device_type_vec_map_[fastcat::FTS_STATE];

  // for(auto dev_state_ptr = vec_state_ptrs.begin(); dev_state_ptr != vec_state_ptrs.end(); dev_state_ptr++){
  // auto wrench_msg = geometry_msgs::msg::Wrench();

  //   std::string device = (*dev_state_ptr)->name;
  //   fastcat::FtsState fts_state = (*dev_state_ptr)->fts_state;

  //   wrench_msg.force.x = fts_state.raw_fx;
  //   wrench_msg.force.y = fts_state.raw_fy;
  //   wrench_msg.force.z = fts_state.raw_fz;

  //   wrench_msg.torque.x = fts_state.raw_tx;
  //   wrench_msg.torque.y = fts_state.raw_ty;
  //   wrench_msg.torque.z = fts_state.raw_tz;

  //   fts_raw_pub_map_[device]->publish(wrench_msg);


  //   wrench_msg.force.x = fts_state.tared_fx;
  //   wrench_msg.force.y = fts_state.tared_fy;
  //   wrench_msg.force.z = fts_state.tared_fz;

  //   wrench_msg.torque.x = fts_state.tared_tx;
  //   wrench_msg.torque.y = fts_state.tared_ty;
  //   wrench_msg.torque.z = fts_state.tared_tz;

  //   fts_tared_pub_map_[device]->publish(wrench_msg);
  //  }

  if (vec_state_ptrs.size() > 0) {
    size_t index = 0;

    fts_states_msg_.header.stamp = publish_time_stamp_;
    for (auto dev_state_ptr = vec_state_ptrs.begin();
         dev_state_ptr != vec_state_ptrs.end(); dev_state_ptr++) {
      // Aggregate FTS state message
      fts_states_msg_.names[index]  = (*dev_state_ptr)->name;
      fts_states_msg_.states[index] = FtsStateToMsg(*dev_state_ptr);
      index++;

      if (enable_ros_wrench_pub_) {
        auto wrench_msg = geometry_msgs::msg::WrenchStamped();

        wrench_msg.header.stamp = publish_time_stamp_;

        std::string       device    = (*dev_state_ptr)->name;
        fastcat::FtsState fts_state = (*dev_state_ptr)->fts_state;

        // Publish Raw FTS
        wrench_msg.header.stamp = publish_time_stamp_;

        wrench_msg.wrench.force.x = fts_state.raw_fx;
        wrench_msg.wrench.force.y = fts_state.raw_fy;
        wrench_msg.wrench.force.z = fts_state.raw_fz;

        wrench_msg.wrench.torque.x = fts_state.raw_tx;
        wrench_msg.wrench.torque.y = fts_state.raw_ty;
        wrench_msg.wrench.torque.z = fts_state.raw_tz;

        fts_raw_pub_map_[device]->publish(wrench_msg);

        // Publish Tared FTS
        wrench_msg.header.stamp = publish_time_stamp_;

        wrench_msg.wrench.force.x = fts_state.tared_fx;
        wrench_msg.wrench.force.y = fts_state.tared_fy;
        wrench_msg.wrench.force.z = fts_state.tared_fz;

        wrench_msg.wrench.torque.x = fts_state.tared_tx;
        wrench_msg.wrench.torque.y = fts_state.tared_ty;
        wrench_msg.wrench.torque.z = fts_state.tared_tz;

        fts_tared_pub_map_[device]->publish(wrench_msg);
      }
    }
  }
}

void Fcat::PublishActuatorStates(){

  // auto act_state_vec = device_type_vec_map_[fastcat::ACTUATOR_STATE];
  // if(act_state_vec.size() > 0){
  //   size_t index = 0;
  //   auto js_msg = sensor_msgs::msg::JointState(); // TODO preallocate

  //   for(auto state_ptr = act_state_vec.begin(); 
  //       state_ptr != act_state_vec.end(); state_ptr++)
  //   {
  //     // Populate ActuatorStates message
  //     actuator_states_msg_.names[index] = (*state_ptr)->name;
  //     actuator_states_msg_.states[index] = ActuatorStateToMsg(*state_ptr);
  //     index++;

  //     // Populate JointState message
  //     js_msg.name.push_back((*state_ptr)->name);
  //     js_msg.position.push_back((*state_ptr)->actuator_state.actual_position);
  //     js_msg.velocity.push_back((*state_ptr)->actuator_state.actual_velocity);
  //   }

  //   actuator_pub_->publish(actuator_states_msg_);

  //   js_msg.header.stamp = now();
  //   if(enable_js_pub_){
  //     joint_state_pub_->publish(js_msg);
  //   }

  // }
  auto& gold_act_state_vec = device_type_vec_map_[fastcat::GOLD_ACTUATOR_STATE];
  auto& platinum_act_state_vec =
      device_type_vec_map_[fastcat::PLATINUM_ACTUATOR_STATE];
  size_t num_actuators =
      gold_act_state_vec.size() + platinum_act_state_vec.size();

  if (num_actuators > 0) {
    size_t index  = 0;
    auto   js_msg = sensor_msgs::msg::JointState();
    js_msg.name.reserve(num_actuators);
    js_msg.position.reserve(num_actuators);
    js_msg.velocity.reserve(num_actuators);

    actuator_states_msg_.header.stamp = publish_time_stamp_;
    js_msg.header.stamp               = publish_time_stamp_;

    for (auto state_ptr : gold_act_state_vec) {
      // Populate ActuatorStates message
      actuator_states_msg_.names[index]  = state_ptr->name;
      actuator_states_msg_.states[index] = ActuatorStateToMsg(state_ptr);
      index++;

      // Populate JointState message
      js_msg.name.push_back(state_ptr->name);
      js_msg.position.push_back(state_ptr->gold_actuator_state.actual_position);
      js_msg.velocity.push_back(state_ptr->gold_actuator_state.actual_velocity);
    }

    for (auto state_ptr : platinum_act_state_vec) {
      // Populate ActuatorStates message
      actuator_states_msg_.names[index]  = state_ptr->name;
      actuator_states_msg_.states[index] = ActuatorStateToMsg(state_ptr);
      index++;

      // Populate JointState message
      js_msg.name.push_back(state_ptr->name);
      js_msg.position.push_back(
          state_ptr->platinum_actuator_state.actual_position);
      js_msg.velocity.push_back(
          state_ptr->platinum_actuator_state.actual_velocity);
    }

    actuator_pub_->publish(actuator_states_msg_);

    js_msg.header.stamp = this->now();
    if (enable_js_pub_) {
      joint_state_pub_->publish(js_msg);
    }
  }
}

void Fcat::PublishEgdStates(){

  auto state_vec = device_type_vec_map_[fastcat::EGD_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    // for(auto state_ptr = state_vec.begin(); 
    //     state_ptr != state_vec.end(); state_ptr++)
    egd_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++)
    {
      egd_states_msg_.names[index] = (*state_ptr)->name;
      egd_states_msg_.states[index] = EgdStateToMsg(*state_ptr);
      index++;
    }
    egd_pub_->publish(egd_states_msg_);
  }
}

void Fcat::PublishThreeNodeThermalModelStates()
{
  // FTS, AtiFts, VirtualFts
  auto vec_state_ptrs =
      device_type_vec_map_[fastcat::THREE_NODE_THERMAL_MODEL_STATE];
  if (vec_state_ptrs.size() > 0) {
    size_t index = 0;

    three_node_thermal_model_states_msg_.header.stamp = publish_time_stamp_;
    for (auto dev_state_ptr = vec_state_ptrs.begin();
         dev_state_ptr != vec_state_ptrs.end(); dev_state_ptr++) {
      three_node_thermal_model_states_msg_.names[index] =
          (*dev_state_ptr)->name;
      three_node_thermal_model_states_msg_.states[index] =
          ThreeNodeThermalModelStateToMsg(*dev_state_ptr);
      index++;
    }
    three_node_thermal_model_pub_->publish(
        three_node_thermal_model_states_msg_);
  }
}

// void Fcat::PublishEl2124States(){

//   auto state_vec = device_type_vec_map_[fastcat::EL2124_STATE];
//   if(state_vec.size() > 0){
//     size_t index = 0;

//     for(auto state_ptr = state_vec.begin(); 
//         state_ptr != state_vec.end(); state_ptr++)
//     {
//       el2124_states_msg_.names[index] = (*state_ptr)->name;
//       el2124_states_msg_.states[index] = El2124StateToMsg(*state_ptr);
//       index++;
//     }
//     el2124_pub_->publish(el2124_states_msg_);
//   }
// }

// void Fcat::PublishEl3208States(){

//   auto state_vec = device_type_vec_map_[fastcat::EL3208_STATE];
//   if(state_vec.size() > 0){
//     size_t index = 0;

//     for(auto state_ptr = state_vec.begin(); 
//         state_ptr != state_vec.end(); state_ptr++)
//     {
//       el3208_states_msg_.names[index] = (*state_ptr)->name;
//       el3208_states_msg_.states[index] = El3208StateToMsg(*state_ptr);
//       index++;
//     }
//     el3208_pub_->publish(el3208_states_msg_);
//   }
// }

// void Fcat::PublishEl3602States(){

//   auto state_vec = device_type_vec_map_[fastcat::EL3602_STATE];
//   if(state_vec.size() > 0){
//     size_t index = 0;

//     for(auto state_ptr = state_vec.begin(); 
//         state_ptr != state_vec.end(); state_ptr++)
//     {
//       el3602_states_msg_.names[index] = (*state_ptr)->name;
//       el3602_states_msg_.states[index] = El3602StateToMsg(*state_ptr);
//       index++;
//     }
//     el3602_pub_->publish(el3602_states_msg_);
//   }
// }

// void Fcat::PublishJedStates(){

//   auto state_vec = device_type_vec_map_[fastcat::JED_STATE];
//   if(state_vec.size() > 0){
//     size_t index = 0;

//     for(auto state_ptr = state_vec.begin(); 
//         state_ptr != state_vec.end(); state_ptr++)
//     {
//       jed_states_msg_.names[index] = (*state_ptr)->name;
//       jed_states_msg_.states[index] = JedStateToMsg(*state_ptr);
//       index++;
//     }
//     jed_pub_->publish(jed_states_msg_);
//   }
// }

void Fcat::PublishEl1008States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL1008_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el1008_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el1008_states_msg_.names[index]  = (*state_ptr)->name;
      el1008_states_msg_.states[index] = El1008StateToMsg(*state_ptr);
      index++;
    }
    el1008_pub_->publish(el1008_states_msg_);
  }
}

void Fcat::PublishEl2124States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL2124_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el2124_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el2124_states_msg_.names[index]  = (*state_ptr)->name;
      el2124_states_msg_.states[index] = El2124StateToMsg(*state_ptr);
      index++;
    }
    el2124_pub_->publish(el2124_states_msg_);
  }
}

void Fcat::PublishEl2809States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL2809_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el2809_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el2809_states_msg_.names[index]  = (*state_ptr)->name;
      el2809_states_msg_.states[index] = El2809StateToMsg(*state_ptr);
      index++;
    }
    el2809_pub_->publish(el2809_states_msg_);
  }
}

void Fcat::PublishEl3104States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL3104_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3104_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el3104_states_msg_.names[index]  = (*state_ptr)->name;
      el3104_states_msg_.states[index] = El3104StateToMsg(*state_ptr);
      index++;
    }
    el3104_pub_->publish(el3104_states_msg_);
  }
}

void Fcat::PublishEl3162States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL3162_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3162_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el3162_states_msg_.names[index]  = (*state_ptr)->name;
      el3162_states_msg_.states[index] = El3162StateToMsg(*state_ptr);
      index++;
    }
    el3162_pub_->publish(el3162_states_msg_);
  }
}

void Fcat::PublishEl3202States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL3202_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3202_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el3202_states_msg_.names[index]  = (*state_ptr)->name;
      el3202_states_msg_.states[index] = El3202StateToMsg(*state_ptr);
      index++;
    }
    el3202_pub_->publish(el3202_states_msg_);
  }
}

void Fcat::PublishEl3208States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL3208_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3208_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el3208_states_msg_.names[index]  = (*state_ptr)->name;
      el3208_states_msg_.states[index] = El3208StateToMsg(*state_ptr);
      index++;
    }
    el3208_pub_->publish(el3208_states_msg_);
  }
}

void Fcat::PublishEl3318States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL3318_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3318_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el3318_states_msg_.names[index]  = (*state_ptr)->name;
      el3318_states_msg_.states[index] = El3318StateToMsg(*state_ptr);
      index++;
    }
    el3318_pub_->publish(el3318_states_msg_);
  }
}

void Fcat::PublishEl3602States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL3602_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3602_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el3602_states_msg_.names[index]  = (*state_ptr)->name;
      el3602_states_msg_.states[index] = El3602StateToMsg(*state_ptr);
      index++;
    }
    el3602_pub_->publish(el3602_states_msg_);
  }
}

void Fcat::PublishEl4102States()
{
  auto state_vec = device_type_vec_map_[fastcat::EL4102_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el4102_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      el4102_states_msg_.names[index]  = (*state_ptr)->name;
      el4102_states_msg_.states[index] = El4102StateToMsg(*state_ptr);
      index++;
    }
    el4102_pub_->publish(el4102_states_msg_);
  }
}

void Fcat::PublishIld1900States()
{
  auto state_vec = device_type_vec_map_[fastcat::ILD1900_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    ild1900_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      ild1900_states_msg_.names[index]  = (*state_ptr)->name;
      ild1900_states_msg_.states[index] = Ild1900StateToMsg(*state_ptr);
      index++;
    }
    ild1900_pub_->publish(ild1900_states_msg_);
  }
}

void Fcat::PublishCommanderStates(){

  auto state_vec = device_type_vec_map_[fastcat::COMMANDER_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    commander_states_msg_.header.stamp = publish_time_stamp_;
    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      commander_states_msg_.names[index] = (*state_ptr)->name;
      commander_states_msg_.states[index] = CommanderStateToMsg(*state_ptr);
      index++;
    }
    commander_pub_->publish(commander_states_msg_);
  }
}

void Fcat::PublishConditionalStates(){

  auto state_vec = device_type_vec_map_[fastcat::CONDITIONAL_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    conditional_states_msg_.header.stamp = publish_time_stamp_;
    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      conditional_states_msg_.names[index] = (*state_ptr)->name;
      conditional_states_msg_.states[index] = ConditionalStateToMsg(*state_ptr);
      index++;
    }
    conditional_pub_->publish(conditional_states_msg_);
  }
}

void Fcat::PublishFaulterStates(){

  auto state_vec = device_type_vec_map_[fastcat::FAULTER_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    faulter_states_msg_.header.stamp = publish_time_stamp_;
    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      faulter_states_msg_.names[index] = (*state_ptr)->name;
      faulter_states_msg_.states[index] = FaulterStateToMsg(*state_ptr);
      index++;
    }
    faulter_pub_->publish(faulter_states_msg_);
  }
}

void Fcat::PublishFilterStates(){

  auto state_vec = device_type_vec_map_[fastcat::FILTER_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    filter_states_msg_.header.stamp = publish_time_stamp_;
    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      filter_states_msg_.names[index] = (*state_ptr)->name;
      filter_states_msg_.states[index] = FilterStateToMsg(*state_ptr);
      index++;
    }
    filter_pub_->publish(filter_states_msg_);
  }
}

void Fcat::PublishFunctionStates(){

  auto state_vec = device_type_vec_map_[fastcat::FUNCTION_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    function_states_msg_.header.stamp = publish_time_stamp_;
    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      function_states_msg_.names[index] = (*state_ptr)->name;
      function_states_msg_.states[index] = FunctionStateToMsg(*state_ptr);
      index++;
    }
    function_pub_->publish(function_states_msg_);
  }
}

void Fcat::PublishPidStates(){

  auto state_vec = device_type_vec_map_[fastcat::PID_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    pid_states_msg_.header.stamp = publish_time_stamp_;
    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      pid_states_msg_.names[index] = (*state_ptr)->name;
      pid_states_msg_.states[index] = PidStateToMsg(*state_ptr);
      index++;
    }
    pid_pub_->publish(pid_states_msg_);
  }
}

void Fcat::PublishSaturationStates(){

  auto state_vec = device_type_vec_map_[fastcat::SATURATION_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    saturation_states_msg_.header.stamp = publish_time_stamp_;
    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      saturation_states_msg_.names[index] = (*state_ptr)->name;
      saturation_states_msg_.states[index] = SaturationStateToMsg(*state_ptr);
      index++;
    }
    saturation_pub_->publish(saturation_states_msg_);
  }
}

void Fcat::PublishSchmittTriggerStates(){

  auto state_vec = device_type_vec_map_[fastcat::SCHMITT_TRIGGER_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    schmitt_trigger_states_msg_.header.stamp = publish_time_stamp_;
    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      schmitt_trigger_states_msg_.names[index] = (*state_ptr)->name;
      schmitt_trigger_states_msg_.states[index] = SchmittTriggerStateToMsg(*state_ptr);
      index++;
    }
    schmitt_trigger_pub_->publish(schmitt_trigger_states_msg_);
  }
}

void Fcat::PublishSignalGeneratorStates(){

  auto state_vec = device_type_vec_map_[fastcat::SIGNAL_GENERATOR_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    signal_generator_states_msg_.header.stamp = publish_time_stamp_;
    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      signal_generator_states_msg_.names[index] = (*state_ptr)->name;
      signal_generator_states_msg_.states[index] = SignalGeneratorStateToMsg(*state_ptr);
      index++;
    }
    signal_generator_pub_->publish(signal_generator_states_msg_);
  }
}

void Fcat::PublishLinearInterpolationStates()
{
  auto state_vec = device_type_vec_map_[fastcat::LINEAR_INTERPOLATION_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    linear_interpolation_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end();
         state_ptr++) {
      linear_interpolation_states_msg_.names[index] = (*state_ptr)->name;
      linear_interpolation_states_msg_.states[index] =
          LinearInterpolationStateToMsg(*state_ptr);
      index++;
    }
    linear_interpolation_pub_->publish(linear_interpolation_states_msg_);
  }
}

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(Fcat)