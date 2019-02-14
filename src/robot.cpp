#include <arl_hw_ethercat/robot.h>

ARLRobot::ARLRobot()
{
  initialized_ = false;
  ec_master_ = new EtherCATMaster();
}

ARLRobot::~ARLRobot()
{
  if (!initialized_)
  {
    ROS_WARN("Robot not initialized, nothing to close");
    return;
  }

  close();
}

void ARLRobot::initialize(ros::NodeHandle nh)
{

  getConfigurationFromParameterServer(nh);

  number_of_muscles_ = driver_config.number_of_controller_boards * 32;


  for (int i = 0; i < number_of_muscles_; i++)
  {
    arl_interfaces::MuscleHandle muscle_handle(muscle_names_[i], &desired_pressures_[i], &current_pressures_[i], &tensions_[i],
                                               &activations_[i], &tensions_filtered_[i], &control_modes_[i]);
    muscle_interface.registerHandle(muscle_handle);
  }

  registerInterface(&muscle_interface);

  //TODO
  if (ec_master_->initialize(driver_config.ethercat_iface_name))
  {
    initialized_ = true;
  }

  ec_master_->write(activations_);
  //Maybe send twice

  emergency_stop = false;
}

void ARLRobot::close()
{
  //TODO
  ec_master_->close();
}

void ARLRobot::read(const ros::Time &time, const ros::Duration &period)
{
  if (!initialized_)
  {
    ROS_WARN_ONCE("Robot not initialized, no data can be read");
    return;
  }

  //TODO read also converts to MPa and Volts
  ec_master_->read(current_pressures_, tensions_);

  //ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time &time, const ros::Duration &period)
{
  if (!initialized_)
  {
    ROS_WARN_ONCE("Robot not initialized, no data can be written");
    return;
  }

  ec_master_->write(activations_);

  //ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}

void ARLRobot::getConfigurationFromParameterServer(ros::NodeHandle nh)
{

  ROS_DEBUG("Reading configuration");

  nh.param<std::string>("/ethercat_iface_name", driver_config.ethercat_iface_name, "eth0");

  nh.param<bool>("/publish_every_rt_jitter", driver_config.publish_every_rt_jitter, false);

  nh.param<bool>("/halt_on_slow_rt_loop", driver_config.halt_on_slow_rt_loop, false);

  nh.param<double>("/min_acceptable_rt_loop_frequency", driver_config.min_acceptable_rt_loop_frequency, 490.0);

  nh.param<int>("/number_of_controller_boards", driver_config.number_of_controller_boards, 0);


  XmlRpc::XmlRpcValue muscle_list;
  nh.getParam("/muscle_list", muscle_list);
  if (muscle_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Parameter muscle_list should be an array");
    return;
  } else {
    ROS_DEBUG("Muscle list of size %d found", muscle_list.size());
  }

  /* define all muscles */
  for (int i = 0; i < muscle_list.size(); ++i)
  {
    if (!muscle_list[i].hasMember("name") || !muscle_list[i].hasMember("initial_value"))
    {
      ROS_ERROR("Definition not complete for muscle %d", i);
      continue;
    }

    try
    {
      //Try to access all muscle fields first and safe them
      std::string name = std::string(muscle_list[i]["name"]);
      double initial_value = muscle_list[i]["initial_value"];

      ROS_DEBUG("Found on server %s at %f", name.c_str(), initial_value);

      muscle_names_.push_back(name);
      desired_pressures_.push_back(initial_value);
      current_pressures_.push_back(0.0);
      tensions_.push_back(0.0);
      activations_.push_back(-0.3);
      tensions_filtered_.push_back(0.0);
      control_modes_.push_back(arl_hw_msgs::MuscleCommand::CONTROL_MODE_BY_ACTIVATION);
      last_activations_.push_back(0.0);
      muscle_index_map_[name] = i;
    }
    catch (...)
    {
      ROS_ERROR("Unable to parse muscle information");
    }
  }

  //Try to access all analog input fields
  XmlRpc::XmlRpcValue analog_inputs;
  if(nh.getParam("/analog_inputs", analog_inputs))
  {
    if (analog_inputs.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Parameter analog_inputs should be an array");
      return;
    } else
    {
      ROS_DEBUG("Analog inputs list of size %d found", analog_inputs.size());
    }

    for (int i = 0; i < analog_inputs.size(); ++i)
    {
      if (!analog_inputs[i].hasMember("name") || !analog_inputs[i].hasMember("controller_channel") ||
          !analog_inputs[i].hasMember("controller_board") || !analog_inputs[i].hasMember("publish_muscle"))
      {
        ROS_ERROR("Either name, channel or port not defined for analog input %d", i);
        continue;
      }

      try
      {
        std::string name = std::string(analog_inputs[i]["name"]);
        analog_input_names_.push_back(name);
        int controller_board = analog_inputs[i]["controller_board"];
        int controller_channel = analog_inputs[i]["controller_channel"];
        analog_input_indicies_.push_back((controller_board * 32) + controller_channel);
        analog_input_values_.push_back(0.0);
        analog_input_publish_.push_back(analog_inputs[i]["publish_muscle"]);
      }
      catch (...)
      {
        ROS_ERROR("Unable to parse analog inputs information");
      }
    }
  }

  ROS_INFO("Robot configuration read from parameter server");
}

unsigned long ARLRobot::getNumberOfMuscles()
{
  return muscle_names_.size();
}

unsigned long ARLRobot::getNumberOfAnalogInputs()
{
  return analog_input_names_.size();
}

struct ARLRobot::muscle_info_t ARLRobot::getMuscleInfo(unsigned long index)
{
  struct muscle_info_t ret = {
      .name = muscle_names_[index],
      .activation = activations_[index],
      .current_pressure = current_pressures_[index],
      .desired_pressure = desired_pressures_[index],
      .tension = tensions_[index],
      .tension_filtered = tensions_filtered_[index],
      .control_mode = control_modes_[index]
  };
  return ret;
}

struct ARLRobot::analog_input_info_t ARLRobot::getAnalogInputInfo(unsigned long index)
{
  struct analog_input_info_t ret = {
      .name = analog_input_names_[index],
      .voltage = analog_input_values_[index]
  };
  return ret;
}

void ARLRobot::updateMuscleValues(arl_hw_msgs::MusculatureCommand musculature_command)
{
  for (arl_hw_msgs::MuscleCommand command : musculature_command.muscle_commands)
  {
    auto muscle_itr = muscle_index_map_.find(command.name);
    if (muscle_itr != muscle_index_map_.end())
    {
      desired_pressures_[muscle_itr->second] = command.pressure;
      activations_[muscle_itr->second] = command.activation;
      control_modes_[muscle_itr->second] = command.control_mode;
    }
  }
}

void ARLRobot::executeEmergencyStop()
{
  for (unsigned int i = 0; i < muscle_names_.size(); i++)
  {
    ec_master_->emergency_stop(i);
    desired_pressures_[i] = 0.0;
    activations_[i] = -1.0;
    control_modes_[i] = arl_hw_msgs::MuscleCommand::CONTROL_MODE_BY_ACTIVATION;
  }
}

void ARLRobot::resetMuscles() {
  for (unsigned int i = 0; i < muscle_names_.size(); i++)
  {
    ec_master_->reset_muscle(i);
    desired_pressures_[i] = 0.0;
    activations_[i] = -1.0;
    control_modes_[i] = arl_hw_msgs::MuscleCommand::CONTROL_MODE_BY_ACTIVATION;
  }
}

void ARLRobot::resetMuscle(std::string name) {
  auto muscle_itr = muscle_index_map_.find(name);
  if (muscle_itr != muscle_index_map_.end())
  {
    ec_master_->reset_muscle(muscle_itr->second);
    desired_pressures_[muscle_itr->second] = 0.0;
    activations_[muscle_itr->second] = -1.0;
    control_modes_[muscle_itr->second] = arl_hw_msgs::MuscleCommand::CONTROL_MODE_BY_ACTIVATION;
  }
}
