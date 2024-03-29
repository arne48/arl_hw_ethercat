#include <arl_hw_ethercat/driver_utils.h>

namespace driver_utils {

  void publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher, statistics_t &driver_stats)
  {
    if (publisher.trylock())
    {
      accumulator_set<double, stats<tag::max, tag::mean> > zero;
      std::vector<diagnostic_msgs::DiagnosticStatus> statuses;
      diagnostic_updater::DiagnosticStatusWrapper status;

      static double max_read = 0, max_write = 0, max_cm = 0, max_loop = 0, max_jitter = 0;
      double avg_read = 0, avg_write = 0, avg_cm = 0, avg_loop = 0, avg_jitter = 0;

      avg_read = extract_result<tag::mean>(driver_stats.read_acc);
      avg_write = extract_result<tag::mean>(driver_stats.write_acc);
      avg_cm = extract_result<tag::mean>(driver_stats.cm_acc);
      avg_loop = extract_result<tag::mean>(driver_stats.loop_acc);
      max_read = std::max(max_read, extract_result<tag::max>(driver_stats.read_acc));
      max_write = std::max(max_write, extract_result<tag::max>(driver_stats.write_acc));
      max_cm = std::max(max_cm, extract_result<tag::max>(driver_stats.cm_acc));
      max_loop = std::max(max_loop, extract_result<tag::max>(driver_stats.loop_acc));
      driver_stats.read_acc = zero;
      driver_stats.write_acc = zero;
      driver_stats.cm_acc = zero;
      driver_stats.loop_acc = zero;

      // Publish average loop jitter
      avg_jitter = extract_result<tag::mean>(driver_stats.jitter_acc);
      max_jitter = std::max(max_jitter, extract_result<tag::max>(driver_stats.jitter_acc));
      driver_stats.jitter_acc = zero;

      status.addf("Max writing roundtrip of controllers (us)", "%.2f", max_write * USEC_PER_SECOND);
      status.addf("Avg writing roundtrip of controllers (us)", "%.2f", avg_write * USEC_PER_SECOND);
      status.addf("Max reading roundtrip of controllers (us)", "%.2f", max_read * USEC_PER_SECOND);
      status.addf("Avg reading roundtrip of controllers (us)", "%.2f", avg_read * USEC_PER_SECOND);
      status.addf("Max Controller Manager roundtrip (us)", "%.2f", max_cm * USEC_PER_SECOND);
      status.addf("Avg Controller Manager roundtrip (us)", "%.2f", avg_cm * USEC_PER_SECOND);
      status.addf("Max Total Loop roundtrip (us)", "%.2f", max_loop * USEC_PER_SECOND);
      status.addf("Avg Total Loop roundtrip (us)", "%.2f", avg_loop * USEC_PER_SECOND);
      status.addf("Max Loop Jitter (us)", "%.2f", max_jitter * USEC_PER_SECOND);
      status.addf("Avg Loop Jitter (us)", "%.2f", avg_jitter * USEC_PER_SECOND);
      status.addf("Control Loop Overruns", "%d", driver_stats.overruns);
      status.addf("Recent Control Loop Overruns", "%d", driver_stats.recent_overruns);
      status.addf("Last Control Loop Overrun Cause", "read: %.2fus, cm: %.2fus, write: %.2fus",
                  driver_stats.overrun_read * USEC_PER_SECOND, driver_stats.overrun_cm * USEC_PER_SECOND,
                  driver_stats.overrun_write * USEC_PER_SECOND);
      status.addf("Last Overrun Loop Time (us)", "%.2f", driver_stats.overrun_loop_sec * USEC_PER_SECOND);
      status.addf("Realtime Loop Frequency", "%.4f", driver_stats.rt_loop_frequency);

      status.name = "Realtime Control Loop";
      if (driver_stats.overruns > 0 && driver_stats.last_overrun < 30)
      {
        if (driver_stats.last_severe_overrun < 30)
          status.level = 1;
        else
          status.level = 0;
        status.message = "Realtime loop used too much time in the last 30 seconds.";
      } else
      {
        status.level = 0;
        status.message = "OK";
      }
      driver_stats.recent_overruns = 0;
      driver_stats.last_overrun++;
      driver_stats.last_severe_overrun++;

      if (driver_stats.rt_loop_not_making_timing)
      {
        status.mergeSummaryf(status.ERROR, "Halting, realtime loop only ran at %.4f Hz", driver_stats.halt_rt_loop_frequency);
      }

      if (driver_stats.emergency_stop_engaged)
      {
        status.mergeSummaryf(status.WARN, "Emergency Stop Engaged");
      }

      statuses.push_back(status);
      publisher.msg_.status = statuses;
      publisher.msg_.header.stamp = ros::Time::now();
      publisher.unlockAndPublish();
    }
  }

  double get_now()
  {
    struct timespec n = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &n);
    return double(n.tv_nsec) / NSEC_PER_SECOND + n.tv_sec;
  }

  void timespecInc(struct timespec *timestamp, int ns)
  {
    timestamp->tv_nsec += ns;
    while (timestamp->tv_nsec >= 1e9)
    {
      timestamp->tv_nsec -= 1e9;
      timestamp->tv_sec++;
    }
  }

  void waitForNextControlLoop(struct timespec timestamp, int sampling_ns)
  {

    timespecInc(&timestamp, sampling_ns);

    struct timespec before = {0, 0};
    clock_gettime(CLOCK_REALTIME, &before);

    timestamp.tv_sec = before.tv_sec;
    timestamp.tv_nsec = (before.tv_nsec / sampling_ns) * sampling_ns;
    timespecInc(&timestamp, sampling_ns);

    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &timestamp, NULL);
  }

  void checkOverrun(statistics_t &driver_stats, double start, double after_read, double after_cm, double after_write,
               int period_int, struct timespec &timestamp)
  {
    struct timespec before = {0, 0};

    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND) > (timestamp.tv_sec + double(timestamp.tv_nsec) / NSEC_PER_SECOND))
    {
      // Total amount of time the loop took to run
      driver_stats.overrun_loop_sec = (before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND) -
                                      (timestamp.tv_sec + double(timestamp.tv_nsec) / NSEC_PER_SECOND);

      // We overran, snap to next "period"
      timestamp.tv_sec = before.tv_sec;
      timestamp.tv_nsec = (before.tv_nsec / period_int) * period_int;
      driver_utils::timespecInc(&timestamp, period_int);

      // initialize overruns
      if (driver_stats.overruns == 0)
      {
        driver_stats.last_overrun = 1000;
        driver_stats.last_severe_overrun = 1000;
      }

      // check for overruns
      if (driver_stats.recent_overruns > 10)
      {
        driver_stats.last_severe_overrun = 0;
      }
      driver_stats.last_overrun = 0;

      driver_stats.overruns++;
      driver_stats.recent_overruns++;
      driver_stats.overrun_read = after_read - start;
      driver_stats.overrun_cm = after_cm - after_read;
      driver_stats.overrun_write = after_write - after_cm;
    }

  }

  void checkSevereRTMiss(double *last_rt_monitor_time, unsigned int *rt_cycle_count, double rt_loop_monitor_period,
                    RTLoopHistory &rt_loop_history,
                    driver_utils::statistics_t &driver_stats, double start, ARLRobot &robot)
  {

    // Realtime loop should run about 1000Hz.
    // Missing timing on a control cycles usually causes a controller glitch and actuators to jerk.
    // When realtime loop misses a lot of cycles controllers will perform poorly and may cause robot to shake.

    ++*rt_cycle_count;
    if ((start - *last_rt_monitor_time) > rt_loop_monitor_period)
    {
      // Calculate new average rt loop frequency
      double rt_loop_frequency = double(*rt_cycle_count) / rt_loop_monitor_period;

      // Use last X samples of frequency when deciding whether or not to halt
      rt_loop_history.sample(rt_loop_frequency);
      double avg_rt_loop_frequency = rt_loop_history.average();

      if (avg_rt_loop_frequency < robot.driver_config.min_acceptable_rt_loop_frequency && robot.driver_config.halt_on_slow_rt_loop)
      {
        driver_stats.halt_rt_loop_frequency = avg_rt_loop_frequency;
        driver_stats.rt_loop_not_making_timing = true;
      }

      driver_stats.rt_loop_frequency = avg_rt_loop_frequency;
      *rt_cycle_count = 0;
      *last_rt_monitor_time = start;
    }
  }

  struct statistics_t init_driver_statistics()
  {
    driver_utils::statistics_t ret;
    ret.overruns = 0;
    ret.recent_overruns = 0;
    ret.last_overrun = 0;
    ret.last_severe_overrun = 0;
    ret.overrun_loop_sec = 0;
    ret.overrun_read = 0;
    ret.overrun_write = 0;
    ret.overrun_cm = 0;
    ret.halt_rt_loop_frequency = 0;
    ret.rt_loop_frequency = 0;
    ret.rt_loop_not_making_timing = false;
    ret.emergency_stop_engaged = false;

    return ret;
  }

}
