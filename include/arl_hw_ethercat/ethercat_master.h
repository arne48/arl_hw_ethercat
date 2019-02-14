#ifndef ARL_HW_ETHERCAT_ETHERCAT_MASTER_H
#define ARL_HW_ETHERCAT_ETHERCAT_MASTER_H

#include <vector>
#include <set>
#include <math.h>
#include <ros/console.h>
#include <arl_hw_ethercat/datatypes.h>
#include <soem/ethercat.h>

#define AD5360_REF_VOLTAGE 5.0
#define CONTROLLER_CHANNELS 32
#define ACTIVATION_OFFSET 0
#define ACTIVATION_LEN 2
#define PRESSURE_OFFSET 0
#define PRESSURE_LEN 2
#define TENSION_OFFSET 64
#define TENSION_LEN 4


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
  void read(std::vector<double> &current_voltages, std::vector<double> &tensions);

  /**
  TODO
   */
  void write(std::vector<double> &activations);

  /**
  TODO
   */
  bool initialize(std::string iface);

  /**
   * Cleanup to close communication device
   */
  void close();

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

private:
  char IOmap[4096];
  int expectedWKC;
  volatile int wkc;
  boolean needlf = FALSE;
  boolean inOP = FALSE;
  int oloop, iloop;

  void change_to_OP();
  void print_slave_details();
  uint16_t transfer_voltage(double voltage);
  double map_to_voltage(double x, double in_min, double in_max, double out_min, double out_max);

};

#endif //ARL_HW_ETHERCAT_ETHERCAT_MASTER_H
