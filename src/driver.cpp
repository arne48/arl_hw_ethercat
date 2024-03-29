#include <cstdio>
#include <csignal>
#include <sys/mman.h>
#include <sys/types.h>
#include <pthread.h>

#include <ros/ros.h>
#include <arl_hw_ethercat/robot.h>
#include <arl_hw_ethercat/rt_history.h>
#include <arl_hw_ethercat/driver_utils.h>
#include <arl_hw_msgs/MusculatureCommand.h>
#include <arl_hw_msgs/AnalogInputArray.h>
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#define RATE 500 //Hz

static ARLRobot robot;

struct RTBuffer {
  arl_hw_msgs::MusculatureCommand musculature_command_;
  arl_hw_msgs::AnalogInputArray analog_inputs_;
};
realtime_tools::RealtimeBuffer<RTBuffer> commands_;
RTBuffer commands_struct_;

void *controlLoop(void *)
{
  struct timespec ts = {0, 0};
  struct timespec tick = {0, 0};

  double last_published = 0, last_loop_start = 0, last_rt_monitor_time = 0;
  unsigned rt_cycle_count = 0;
  unsigned long loop_count = 0;
  double rt_loop_monitor_period = 0.6 / 3;
  RTLoopHistory rt_loop_history(3, RATE);
  driver_utils::statistics_t driver_stats = driver_utils::init_driver_statistics();

  ros::NodeHandle nh;

  robot.initialize(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> publisher(nh, "/diagnostics", 2);
  realtime_tools::RealtimePublisher<std_msgs::Float64> *rtpublisher =
    new realtime_tools::RealtimePublisher<std_msgs::Float64>(nh, "/realtime_jitter", 2);
  realtime_tools::RealtimePublisher<arl_hw_msgs::MusculatureState> musculature_state_publisher(nh, "/musculature/state", 2);
  realtime_tools::RealtimePublisher<arl_hw_msgs::AnalogInputArray> analog_input_publisher(nh, "/sensor_readings/analog_inputs", 2);

  commands_struct_.musculature_command_ = arl_hw_msgs::MusculatureCommand();
  commands_.initRT(commands_struct_);

  // Publish one-time before entering real-time to pre-allocate message vectors
  driver_utils::publishDiagnostics(publisher, driver_stats);

  //Set update rate with ROS datatype in HZ
  ros::Rate rate(RATE);

  //Check if RTC is available
  if (clock_gettime(CLOCK_REALTIME, &ts) != 0)
  {
    ROS_FATAL("Failed to poll real-time clock!");
  }

  //Time for use in controllers
  ros::Time
    last(ts.tv_sec, ts.tv_nsec),
    now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  // Get the next full second to start control loop at
  tick.tv_sec = tick.tv_sec;
  tick.tv_nsec = (tick.tv_nsec / int(rate.expectedCycleTime().toNSec()) + 1) * int(rate.expectedCycleTime().toNSec());
  clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);


  last_published = driver_utils::get_now();
  last_rt_monitor_time = driver_utils::get_now();
  last_loop_start = driver_utils::get_now();

  while (nh.ok())
  {

    // Check for Emergency Halt
    driver_stats.emergency_stop_engaged = robot.emergency_stop;

    // Read received musculature command from non-rt context
    commands_struct_ = *(commands_.readFromRT());

    // Update robot muscle value according to command
    robot.updateMuscleValues(commands_struct_.musculature_command_);

    double start = 0, after_read = 0, after_cm = 0, after_write = 0;

    // If emergency stop IS engaged then just resend command to blow-off muscles.
    // In this case not all actions are executed on the controllers anymore
    if (driver_stats.emergency_stop_engaged)
    {
      start = driver_utils::get_now();
      after_read = start;
      after_cm = start;
      after_write = start;

      robot.executeEmergencyStop();
      cm.update(now, period);

      double end = driver_utils::get_now();
      if ((end - last_published) > 1.0)
      {
        driver_utils::publishDiagnostics(publisher, driver_stats);
        last_published = end;
      }


    // If emergency stop IS NOT engaged proceed as usual with all updates
    // and controller actions
    } else
    {
      // Track how long the actual loop takes
      double this_loop_start = driver_utils::get_now();
      driver_stats.loop_acc(this_loop_start - last_loop_start);
      last_loop_start = this_loop_start;

      // Get the time / period
      if (!clock_gettime(CLOCK_REALTIME, &ts))
      {
        now.sec = ts.tv_sec;
        now.nsec = ts.tv_nsec;
        period = now - last;
        last = now;
      } else
      {
        ROS_FATAL("Failed to poll real-time clock!");
        break;
      }

      //Main update
      start = driver_utils::get_now();

      robot.read(now, period);
      after_read = driver_utils::get_now();

      cm.update(now, period);
      after_cm = driver_utils::get_now();

      //Just write new data to muscle controllers as long as the rt loop is stable
      if (!driver_stats.rt_loop_not_making_timing)
      {
        robot.write(now, period);
      }
      after_write = driver_utils::get_now();

      //Accumulate section's durations of update cycle
      driver_stats.read_acc(after_read - start);
      driver_stats.cm_acc(after_cm - after_read);
      driver_stats.write_acc(after_write - after_cm);

      double end = driver_utils::get_now();

      // Publishing diagnostics information if 1 sec has passed
      if ((end - last_published) > 1.0)
      {
        driver_utils::publishDiagnostics(publisher, driver_stats);
        last_published = end;
      }

      // Publishing musculature state and analog inputs message
      if (loop_count % 5 == 0)
      {
        if (musculature_state_publisher.trylock())
        {
          musculature_state_publisher.msg_.header.stamp = ros::Time::now();
          musculature_state_publisher.msg_.header.frame_id = "0";

          musculature_state_publisher.msg_.muscle_states.clear();
          unsigned long muscle_number = robot.getNumberOfMuscles();
          for (unsigned long i = 0; i < muscle_number; i++)
          {
            struct ARLRobot::muscle_info_t muscle_info = robot.getMuscleInfo(i);
            arl_hw_msgs::Muscle muscle_msg;
            muscle_msg.name = muscle_info.name;
            muscle_msg.activation = muscle_info.activation;
            muscle_msg.current_pressure = muscle_info.current_pressure;
            muscle_msg.desired_pressure = muscle_info.desired_pressure;
            muscle_msg.tension = muscle_info.tension;
            muscle_msg.tension_filtered = muscle_info.tension_filtered;
            muscle_msg.control_mode = muscle_info.control_mode;
            musculature_state_publisher.msg_.muscle_states.push_back(muscle_msg);
          }
          musculature_state_publisher.unlockAndPublish();
        }

        if (analog_input_publisher.trylock())
        {
          analog_input_publisher.msg_.header.stamp = ros::Time::now();
          analog_input_publisher.msg_.header.frame_id = "0";

          analog_input_publisher.msg_.inputs.clear();
          unsigned long analog_input_number = robot.getNumberOfAnalogInputs();
          for (unsigned long i = 0; i < analog_input_number; i++)
          {
            struct ARLRobot::analog_input_info_t analog_input_info = robot.getAnalogInputInfo(i);
            arl_hw_msgs::AnalogInput input_msg;
            input_msg.name = analog_input_info.name;
            input_msg.voltage = analog_input_info.voltage;
            analog_input_publisher.msg_.inputs.push_back(input_msg);
          }

          analog_input_publisher.unlockAndPublish();
        }

      }

      driver_utils::checkSevereRTMiss(&last_rt_monitor_time, &rt_cycle_count, rt_loop_monitor_period,
                                      rt_loop_history, driver_stats, start, robot);
      loop_count++;
    }

    // Compute end of next period
    int period_int = int(rate.expectedCycleTime().toNSec());
    driver_utils::timespecInc(&tick, period_int);

    driver_utils::checkOverrun(driver_stats, start, after_read, after_cm, after_write, period_int, tick);

    //Wait for the loop to start and maintain fixed rate
    driver_utils::waitForNextControlLoop(tick, int(rate.expectedCycleTime().toNSec()));

    // Calculate RT loop jitter
    struct timespec after = {0, 0};
    clock_gettime(CLOCK_REALTIME, &after);
    double jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec - tick.tv_nsec) / NSEC_PER_SECOND);
    driver_stats.jitter_acc(jitter);

    // Publish realtime loops current jitter
    if (robot.driver_config.publish_every_rt_jitter && rtpublisher->trylock())
    {
      rtpublisher->msg_.data = jitter;
      rtpublisher->unlockAndPublish();
    }
  }

  delete rtpublisher;
  robot.close();

  return 0;
}

bool resetMusclesService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  robot.resetMuscles();
  return true;
}

bool emergencyStopService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  robot.emergency_stop = req.data;
  return true;
}

void musculatureCommandCallback(const arl_hw_msgs::MusculatureCommand::ConstPtr& msg)
{
  commands_struct_.musculature_command_.muscle_commands = msg->muscle_commands;
  commands_struct_.musculature_command_.header = msg->header;
  commands_.writeFromNonRT(commands_struct_);
}

int main(int argc, char **argv)
{

  // Keep the kernel from swapping us out
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
  {
    printf("mlockall failed\n");
    exit(EXIT_FAILURE);
  }

  //Initialize ROS communication
  ros::init(argc, argv, "arl_driver_node");
  ros::NodeHandle nh;

  ros::ServiceServer reset = nh.advertiseService("/reset_muscles", resetMusclesService);
  ros::ServiceServer halt = nh.advertiseService("/emergency_stop", emergencyStopService);
  ros::Subscriber musculature_sub = nh.subscribe("/musculature/command", 2, musculatureCommandCallback);

  //Start thread

  // Set real-time scheduler parameters
  pthread_t controlThread;
  pthread_attr_t controlThreadAttr;
  struct sched_param thread_param;
  int ret;

  ret = pthread_attr_init(&controlThreadAttr);
  if (ret)
  {
    printf("init pthread attributes failed\n");
    exit(EXIT_FAILURE);
  }

  ret = pthread_attr_setschedpolicy(&controlThreadAttr, SCHED_FIFO);
  if (ret)
  {
    printf("pthread setschedpolicy failed\n");
    exit(EXIT_FAILURE);
  }

  thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  ret = pthread_attr_setschedparam(&controlThreadAttr, &thread_param);
  if (ret)
  {
    printf("pthread setschedparam failed\n");
    exit(EXIT_FAILURE);
  }

  ret = pthread_attr_setinheritsched(&controlThreadAttr, PTHREAD_EXPLICIT_SCHED);
  if (ret)
  {
    printf("pthread setinheritsched failed\n");
    exit(EXIT_FAILURE);
  }

  // Start control thread
  ret = pthread_create(&controlThread, &controlThreadAttr, controlLoop, NULL);
  if (ret)
  {
    ROS_FATAL("Unable to create control thread: rv = %d", ret);
    exit(EXIT_FAILURE);
  }

  ros::spin();

  ret = pthread_join(controlThread, NULL);
  if (ret)
  {
    printf("join pthread failed\n");
  }
}
