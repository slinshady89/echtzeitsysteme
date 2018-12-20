#ifndef TIME_PROFILING_HPP
#define TIME_PROFILING_HPP

// comment this line to not use time profiling
#define USE_TIMER

#ifdef USE_TIMER
    #define TIMER_INIT ros::Time startTime; ros::Time endTime; ros::Duration dur; float time;
    #define TIMER_START startTime = ros::Time::now();
    #define TIMER_STOP endTime = ros::Time::now();
    #define TIMER_EVALUATE(message) dur = endTime - startTime; \
                                int millis = dur.toNSec / 1000000; \
                                ROS_INFO("TIME: message ---> %f ms", millis);


#else
    #define TIMER_INIT
    #define TIMER_START
    #define TIMER_STOP
    #define TIMER_EVALUATE(message)
#endif

#endif // TIME_PROFILING_HPP