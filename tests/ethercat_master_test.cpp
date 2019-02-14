#include <cstdio>
#include <cinttypes>
#include <iostream>
#include <soem/ethercat.h>

char IOmap[4096];
int expectedWKC;
volatile int wkc;
boolean needlf = FALSE;
boolean inOP = FALSE;

int oloop, iloop;

static inline void send_data()
{
  if (ec_slave[0].state == EC_STATE_OPERATIONAL )
  {
    inOP = TRUE;
    /* cyclic loop */
    for(int i = 1; i <= 10; i++)
    {

      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
      if(wkc >= expectedWKC)
      {
        printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

        for(int j = 0 ; j < oloop; j++)
        {
          printf(" %2.2x", *(ec_slave[0].outputs + j));
          *(ec_slave[0].outputs + j) = 0xF0;
        }

        printf(" I:");
        for(int j = 0 ; j < iloop; j+=2)
        {
          printf(" %d", *(ec_slave[0].inputs + j) | *(ec_slave[0].inputs + j+1) << 8);
        }
        printf(" T:%" PRId64 "\r",ec_DCtime);
        needlf = TRUE;
      }
      //osal_usleep(50000000);

    }
    printf("\n");
    inOP = FALSE;
  }
}

static inline void switch_into_operational_state()
{
  printf("Request operational state for all slaves\n");
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
    printf("Operational state reached for all slaves.\n");
  }
}

static inline void switch_into_init_state()
{
  printf("Request init state for all slaves\n");
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
}

static inline void print_slave_details()
{
  oloop = ec_slave[0].Obytes;
  if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
  iloop = ec_slave[0].Ibytes;
  if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
  printf("Segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  printf("Calculated workcounter %d\n", expectedWKC);
}

int main(int argc, char **argv)
{

  if(!ec_init("enp0s31f6"))
  {
    std::cout << "Couldn't start EtherCat master! (try as root)" << std::endl;
    return -1;
  }

  std::cout << "EtherCat master initialized!" << std::endl;
  if (ec_config_init(FALSE) <= 0)
  {
    std::cout << "No slaves found!" << std::endl;
    return -1;
  }

  printf("SOEM found and configured %d slaves\n", ec_slavecount);
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
  {
    printf("Man: %8.8x ID: %8.8x Rev: %8.8x %s\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev, "Slave");
  }

  ec_config_map(&IOmap);
  ec_configdc();

  if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP)
  {
    fprintf(stderr, "Could not set EC_STATE_SAFE_OP\n");
    return -1;
  }

  print_slave_details();
  switch_into_operational_state();
  send_data();
  switch_into_init_state();
  ec_close();
}