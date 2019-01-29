#ifndef ARL_HW_ETHERCAT_ETHERCAT_MASTER_H
#define ARL_HW_ETHERCAT_ETHERCAT_MASTER_H

#include <vector>
#include <set>
#include <ros/console.h>
#include <arl_hw_ethercat/datatypes.h>
#include <soem/ethercat.h>

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
  TODO
   */
  bool read(std::vector<double> &current_pressures, std::vector<double> &tensions, std::vector<double> &tensions_filtered,
            std::vector<double> &analog_input_values, std::vector<int> analog_input_indicies);

  /**
  TODO
   */
  bool write(std::vector<double> &activations);

  /**
  TODO
   */
  bool initialize(std::string iface);

  /**
   * Cleanup to close communication device
   * @return success of command
   */
  bool close();


  /**
   * Blows off air from muscle
   * @param muscle to stop
   */
  void emergency_stop(int muscle);

  /**
   * Resets muscle and blows off air
   * @param muscle
   */
  void reset_muscle(int muscle);

};

#endif //ARL_HW_ETHERCAT_ETHERCAT_MASTER_H
