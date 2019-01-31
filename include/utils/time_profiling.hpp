#ifndef TIME_PROFILING_HPP
#define TIME_PROFILING_HPP

/**
 * These macros can be used to measure the duration of code fragments in a ROS program.
 * TIMER_INIT must be called exactly once in a scope before the other commands to setup
 * measurement variables.
 *
 * If USE_TIMER is not defined, all macros will have no effect and just removed by the preprocessor.
 */

// comment this line to not use time profiling
#define USE_TIMER

#ifdef USE_TIMER
    #define TIMER_INIT ros::Time startTime; ros::Time endTime; float millis;
    #define TIMER_START startTime = ros::Time::now();
    #define TIMER_STOP endTime = ros::Time::now();
    // prints the measured time elapsed between TIMER_START and TIMER_STOP to the ROS console
    #define TIMER_EVALUATE(message) millis = (endTime.toSec() - startTime.toSec()) * 1000.0; \
                                    ROS_INFO("TIME: [%s] ---> %f ms", #message, millis);


#else
    #define TIMER_INIT
    #define TIMER_START
    #define TIMER_STOP
    #define TIMER_EVALUATE(message)
#endif

#endif // TIME_PROFILING_HPP