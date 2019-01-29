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
 * Reads current robot state from hardware
 * @param status_vec output parameter
 * @param pressure_controllers
 * @param tension_controllers
 * @return success of command
 */
bool EtherCATMaster::read(std::vector<arl_datatypes::muscle_status_data_t> &muscle_status_vec, std::vector<std::pair<int, int> > pressure_controllers,
                  std::vector<std::pair<int, int> > tension_controllers, std::vector<arl_datatypes::analog_input_status_data_t> &analog_input_status_vec,
                  std::vector<std::pair<int, int> > analog_inputs_controllers)
{
  return true;
}

/**
 * Writes robot command to hardware
 * @param command_vec command to issue to hardware
 * @return success of command
 */
bool EtherCATMaster::write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec)
{
  return true;
}

/**
 * Initialize communication device
 * @param pressure_controllers
 * @param tension_controllers
 * @return success of command
 */
bool EtherCATMaster::initialize(std::vector<std::pair<int, int> > pressure_controllers,
                        std::vector<std::pair<int, int> > tension_controllers,
                        std::vector<std::pair<int, int> > analog_inputs_controllers)
{
  return true;
}

/**
 * Cleanup to close communication device
 * @return success of command
 */
bool EtherCATMaster::close()
{
  return true;
}

/**
 * Blows off air from muscle
 * @param muscle port of muscle to stop
 */
void EtherCATMaster::emergency_stop(std::pair<int, int> muscle)
{

}

/**
 * Resets muscle and blows off air
 * @param muscle
 */
void EtherCATMaster::reset_muscle(std::pair<int, int> muscle)
{
  
}