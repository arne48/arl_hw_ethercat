#include <arl_hw_ethercat/ethercat_master.h>

EtherCATMaster::EtherCATMaster()
{

}

/**
 * Destructor
 */
EtherCATMaster::~EtherCATMaster()
{

}

/**
TODO
 */
bool EtherCATMaster::read(std::vector<double> &current_pressures, std::vector<double> &tensions, std::vector<double> &tensions_filtered,
          std::vector<double> &analog_input_values, std::vector<int> analog_input_indicies)
{
  return true;
}

/**
TODO
 */
bool EtherCATMaster::write(std::vector<double> &activations)
{
  return true;
}

/**
  TODO
 */
bool EtherCATMaster::initialize(std::string iface)
{
  if(!ec_init(iface.c_str()))
  {
    ROS_ERROR("Couldn't start EtherCAT master! (try as root)");
    return false;
  }

  if (ec_config_init(FALSE) <= 0)
  {
    ROS_ERROR("EtherCAT master couldn't find slaves!");
    return false;
  }

  ROS_INFO("EtherCAT interface initialized.");
  return true;
}

/**
 * Cleanup to close communication device
 * @return success of command
 */
bool EtherCATMaster::close()
{
  ec_close();
  ROS_INFO("EtherCAT interface closed.");
  return true;
}

/**
 * Blows off air from muscle
 * @param muscle to stop
 */
void EtherCATMaster::emergency_stop(int muscle)
{

}

/**
 * Resets muscle and blows off air
 * @param muscle
 */
void EtherCATMaster::reset_muscle(int muscle)
{

}
