#pragma once

#include <atomic>
#include <time.h>
#include <linux/rtc.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <string>


class TimeHandler{

public:

    TimeHandler(std::string rtc_path = "/dev/rtc0");    //< Constructor. Will try to get unix timestamp from NTP server and initialize RTC

    int GetTime(time_t * unix_time);    //< Get time in the form of Unix timestamp. The timestamp will be taken from RTC chip
    int SetTime(time_t unix_time);      //< Set the time of RTC (Hardware Clock time). 
    int AdjustTime(void);               //< Check the difference between the System Clock and Hardware Clock (RTC). Adjust if needed. 

private:
    int fd;
    char const * path_to_rtc;

};