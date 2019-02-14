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
void EtherCATMaster::read(std::vector<double> &current_voltages, std::vector<double> &tensions)
{
  /*for (int i = 1; i <= ec_slavecount; i++)
  {

  }*/

  ROS_INFO("Trying to read data");
  if (ec_slave[0].state == EC_STATE_OPERATIONAL )
  {
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if (wkc >= expectedWKC)
    {
      ROS_INFO(" I:");
      for (int j = PRESSURE_OFFSET; j < CONTROLLER_CHANNELS * PRESSURE_LEN ; j+=PRESSURE_LEN)
      {
        ROS_INFO("Pressure %d: %04X", j/PRESSURE_LEN, ec_slave[0].inputs[j] | ec_slave[0].inputs[j+1] << 8);
      }

      for (int j = TENSION_OFFSET; j < TENSION_OFFSET + (CONTROLLER_CHANNELS * TENSION_LEN); j+=TENSION_LEN)
      {
        ROS_INFO("Tension %d: %04X", (j-TENSION_OFFSET)/TENSION_LEN, ec_slave[0].inputs[j] | ec_slave[0].inputs[j+1] << 8 | ec_slave[0].inputs[j+2] << 16 | ec_slave[0].inputs[j+3] << 24);
      }
    }
  }
}

void EtherCATMaster::write(std::vector<double> &activations)
{
  //ROS_INFO("Trying to write data");
  if (ec_slave[0].state == EC_STATE_OPERATIONAL )
  {
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if (wkc >= expectedWKC)
    {
      for (int slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
      {
        for (int output_idx = ACTIVATION_OFFSET; output_idx < CONTROLLER_CHANNELS * ACTIVATION_LEN; output_idx += ACTIVATION_LEN)
        {
          //ROS_INFO("Activation %d: %04X", output_idx / ACTIVATION_LEN, ec_slave[slave_idx].outputs[output_idx] | ec_slave[slave_idx].outputs[output_idx + 1] << 8);
          double activation = activations[((slave_idx-1)*CONTROLLER_CHANNELS) + (output_idx / ACTIVATION_LEN)];
          uint16_t value = transfer_voltage(map_to_voltage(activation, -1, 1, 0, (AD5360_REF_VOLTAGE * 2)));
          ec_slave[slave_idx].outputs[output_idx] = (uint8_t) (value & 0x00FF);
          ec_slave[slave_idx].outputs[output_idx + 1] = (uint8_t) ((value & 0xFF00) >> 8);
        }
      }
    }
  } else
  {
    ROS_ERROR("EtherCAT: Not all slaves are operational");
  }
}

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

  ROS_INFO("SOEM found and configured %d slaves", ec_slavecount);
  for (int cnt = 1; cnt <= ec_slavecount; cnt++)
  {
    ROS_INFO("Man: %8.8x ID: %8.8x Rev: %8.8x %s", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev, "Slave");
  }

  ec_config_map(&IOmap);
  ec_configdc();

  if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP)
  {
    ROS_ERROR("Could not set EC_STATE_SAFE_OP");
    return false;
  }

  print_slave_details();
  change_to_OP();

  ROS_INFO("EtherCAT interface initialized.");
  return true;
}

/**
 * Cleanup to close communication device
 * @return success of command
 */
void EtherCATMaster::close()
{
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  ec_close();
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

void EtherCATMaster::change_to_OP()
{
  ROS_INFO("Request operational state for all slaves");
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  /* send one valid process data to make outputs in slaves happy*/
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  /* request OP state for all slaves */
  ec_writestate(0);

  int chk = 40;
  /* wait for all slaves to reach OP state */
  do
  {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 500000);
  }
  while (chk-- && (ec_slave[1].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state == EC_STATE_OPERATIONAL )
  {
    ROS_INFO("Operational state reached for all slaves.");
  }
}

void EtherCATMaster::print_slave_details()
{
  oloop = ec_slave[0].Obytes;
  if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
  iloop = ec_slave[0].Ibytes;
  if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
  ROS_INFO("Segments : %d : %d %d %d %d",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  ROS_INFO("Calculated workcounter %d", expectedWKC);
}

uint16_t EtherCATMaster::transfer_voltage(double voltage)
{
  if (voltage == 0.0)
  {
    return (uint16_t)pow(2, 16) / 2;
  } else if (voltage <= (2 * AD5360_REF_VOLTAGE) * -1)
  {
    return 0;
  } else if (voltage >= (2 * AD5360_REF_VOLTAGE))
  {
    return (uint16_t)pow(2, 16) - 1;
  } else
  {
    uint16_t base = pow(2, 16) / 2;
    uint16_t value = fabs(voltage) / ((AD5360_REF_VOLTAGE * 2) / pow(2, 15));

    if (voltage > 0)
    {
      return base + value;
    } else
    {
      return base - value;
    }

  }
}

double EtherCATMaster::map_to_voltage(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
