#pragma once
#include <iostream>
#include <event2/event.h>
#include <sstream>

namespace util
{
    enum class ErrorStatus 
    {
        Success,
        Failed,
        NumOfStatus
    };
    void evtimer_sec_add(struct event *event, time_t sec);
}

