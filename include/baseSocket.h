#pragma once
#include "util.h"
#include <string>
#include <sys/socket.h>
#include "event2/event.h"
#include "event2/buffer.h"
#include "event2/bufferevent.h"
#include "baseEvent.h"

using namespace std;


namespace socketUtil
{
    enum class BevStatus 
    {
        Connected,
        Closed,
        Failed
    };
}

class SocketBase
{
    private:

        int fd;
        struct bufferevent *bev;

    public:
        struct sockaddr_in *sin;
        socketUtil::BevStatus baseSocketConnected;

        SocketBase(string _server, int _port);
        SocketBase(){};
        ~SocketBase();
        int getFd();
        util::ErrorStatus write(const void *data, size_t size);
        util::ErrorStatus connectSocket(void (*_readCb)(struct bufferevent *, void *),
                                         void (*_eventCb)(struct bufferevent* UNUSED,
                                         short, void*), void *arg);
                                         
        void ReConnectSocket(void (*_readCb)(struct bufferevent *, void *),
                                         void (*_eventCb)(struct bufferevent* UNUSED,
                                         short, void*), void *arg);
        void reconnectSocket();
        void setBuffereventNull();
        void setBufferevent(struct bufferevent* _bev);
        void closeBev();

        struct bufferevent* createBuffevent();
        struct sockaddr_in* getSockAddrPort() const
        {
            return sin;
        }
        struct bufferevent* getBufferevent() const
        {
            return bev;
        }


};

