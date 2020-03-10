#pragma once
#include <string>
#include <sys/socket.h>
#include <event.h>
#include <iostream>

class baseEvent
{
    private:
        static struct event_base *ev_base;
        static std::string nameOfBase;


    public:
        baseEvent();
        ~baseEvent();
        static void initBase();
        static void dispatch_event();
        static struct event_base* get_baseEvent();
        static std::string get_name();
        

};