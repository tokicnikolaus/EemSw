#include "util.h"

void util::evtimer_sec_add(struct event *event, time_t sec)
    {
        struct timeval tv;
        tv.tv_sec = sec;
        tv.tv_usec = 0;
        evtimer_add(event, &tv);
    };
