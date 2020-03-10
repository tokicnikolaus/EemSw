#include "baseEvent.h"

using namespace std;

string baseEvent::nameOfBase = "Nikola";
event_base* baseEvent::ev_base = NULL;


struct event_base*
baseEvent::get_baseEvent()
{
    return ev_base;
}

std::string
baseEvent::get_name()
{
    return nameOfBase;
}

baseEvent::baseEvent()
{}

baseEvent::~baseEvent()
{}

void
baseEvent::initBase()
{
    if (!ev_base)
    {
        baseEvent::ev_base = event_base_new();
    }
}

void
baseEvent::dispatch_event()
{
    if(ev_base)
    {
        event_base_dispatch(ev_base);
    }
}
