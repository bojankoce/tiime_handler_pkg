#include <stdio.h>
#include "time_handler.hpp"


TimeHandler::TimeHandler(const char * rtc_path){
    printf("TimeHandler constructor!\n");
    this->path_to_rtc = rtc_path;
    //TODO(bojankoce): Get the Unix Timestamp from NTP server and set RTC time accordingly
}

int TimeHandler::GetTime(time_t * unix_time){
    int ret = 0; //TODO(bojankoce): Define Error Codes

    rtc_time rtc_time_;
    struct tm timeinfo;
    
    fd = open(this->path_to_rtc, O_RDONLY);
    ret = ioctl(fd, RTC_RD_TIME, &rtc_time_);
    close(fd);    
    
    std::memcpy(&timeinfo, &rtc_time_, sizeof(rtc_time_));
    *unix_time = mktime(&timeinfo); 

    return ret;
}

int TimeHandler::SetTime(time_t unix_time){
    int ret = 0; 
    rtc_time rtc_time_;
    
    tm * tm_local = localtime(&unix_time);
    std::memcpy(&rtc_time_, tm_local, sizeof(rtc_time_));
    
    fd = open(this->path_to_rtc, O_RDWR);
    ret = ioctl(fd, RTC_SET_TIME, &rtc_time_);
    close(fd);

    return ret;
}