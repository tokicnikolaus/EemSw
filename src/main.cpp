#include <stdio.h> 
#include <string.h>
#include <iostream>
#include <arpa/inet.h>
#include "baseSocket.h"
#include "baseEvent.h"
#include "EEM.h"
using namespace std;

// baseEvent::nameOfBase = "Nikkkkkk";

void _readCb(struct bufferevent* bev, void *arg)
{
    std::cout << "readCB" << endl;
}

void
eventCb(struct bufferevent *bev, short events, void *arg)
{
    std::cout << "Error handler" << endl;
}
//Multiple definition of main
int main(int argc, char const *argv[]) 
{ 
    baseEvent::initBase();
    std::string server = "192.168.100.100";
    // server = argv[1];
    int port = 2000;
    // port = atoi(argv[2]);

    Eem Vertiv = Eem(server, port);
    Vertiv.connect();



    baseEvent::dispatch_event();
    while(1){}
    return 0;
}