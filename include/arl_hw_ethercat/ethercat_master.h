#ifndef ARL_HW_ETHERCAT_ETHERCAT_MASTER_H
#define ARL_HW_ETHERCAT_ETHERCAT_MASTER_H

#include <vector>
#include <set>
#include <arl_hw_ethercat/datatypes.h>

class EtherCATMaster {
public:
  /**
   * Default Constructor
   */
  EtherCATMaster();

  /**
   * Destructor
   */
  ~EtherCATMaster();

  /**
   * Reads current robot state from hardware
   * @param status_vec output parameter
   * @param pressure_controllers
   * @param tension_controllers
   * @return success of command
   */
  bool read(std::vector<arl_datatypes::muscle_status_data_t> &muscle_status_vec, std::vector<std::pair<int, int> > pressure_controllers,
                    std::vector<std::pair<int, int> > tension_controllers, std::vector<arl_datatypes::analog_input_status_data_t> &analog_input_status_vec,
                    std::vector<std::pair<int, int> > analog_inputs_controllers);

  /**
   * Writes robot command to hardware
   * @param command_vec command to issue to hardware
   * @return success of command
   */
  bool write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec);

  /**
   * Initialize communication device
   * @param pressure_controllers
   * @param tension_controllers
   * @return success of command
   */
  bool initialize(std::vector<std::pair<int, int> > pressure_controllers,
                          std::vector<std::pair<int, int> > tension_controllers,
                          std::vector<std::pair<int, int> > analog_inputs_controllers);

  /**
   * Cleanup to close communication device
   * @return success of command
   */
  bool close();


  /**
   * Blows off air from muscle
   * @param muscle port of muscle to stop
   */
  void emergency_stop(std::pair<int, int> muscle);

  /**
   * Resets muscle and blows off air
   * @param muscle
   */
  void reset_muscle(std::pair<int, int> muscle);

};

#endif //ARL_HW_ETHERCAT_ETHERCAT_MASTER_H
