#pragma once
#include "util.h"
#include "baseSocket.h"
#include "EemReq.h"
#include <vector>
extern void EEM_Init(void);

using namespace std;

// typedef  util::ErrorStatus (EemReq::*eem_callback)(char *, size_t);

enum class EemState 
    {
        EEM_INACTIVE,
        EEM_CONNECTING,
        EEM_CONNECTED
    };

class Eem
{   
    public:
        Eem(string _server, int _port);
        ~Eem();
        vector<EemReq> request_queue;
        SocketBase *EemSocket;
        util::ErrorStatus connect();
        // util::ErrorStatus connect_timeout();
        util::ErrorStatus write(const void *data, size_t size);
        util::ErrorStatus sendNextReq();
        static void readCb(struct bufferevent *bev, void *arg);
        static void eventCb(struct bufferevent *bev, short events, void *arg);
        static void connect_timeout(int fd , short what , void *arg);

        void close();
        EemState eemStatus;
        struct event *connect_timeout_ev;

    private:
        EemReq eemReq;
};


class EemDevice
{
    public:
        int l;


};

