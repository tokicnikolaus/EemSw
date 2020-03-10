#include <iostream>
#include <string>
#include <cstring>
#include "EEM.h"
#include "EemReq.h"
#include <string.h>
#include <vector>
#include <type_traits>

using namespace std;


void
EEM_Init(void)
{
    std::cout << "EEM Init!!"<<std::endl;
}

Eem::Eem(string _server, int _port) : eemStatus(EemState::EEM_INACTIVE)
{
    EemSocket = new SocketBase(_server, _port);
    connect_timeout_ev = evtimer_new(baseEvent::get_baseEvent(), connect_timeout, this);

    
    if (eemStatus == EemState::EEM_INACTIVE && connect_timeout_ev)
    {
        util::evtimer_sec_add(connect_timeout_ev, 5);
    }
}

Eem::~Eem()
{
    try
    {
        cout << "Eem Destructor called!" << endl;
        delete EemSocket;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

util::ErrorStatus
Eem::connect()
{
    cout << "Eem connecting!" << endl;
    EemSocket->baseSocketConnected = socketUtil::BevStatus::Closed;
    return EemSocket->connectSocket(this->readCb, this->eventCb, this);

}

void
Eem::connect_timeout(int fd , short what , void *arg)
{
    Eem *self = static_cast<Eem*>(arg);
    if (self->eemStatus==EemState::EEM_INACTIVE)
    {
        self->EemSocket->setBuffereventNull();
        cout << "Eem connect again!" << endl;
        self->connect();
    }
}

void
Eem::close()
{
    EemSocket->closeBev();
    // eemReq.cleanBufferevent()
    eemStatus=EemState::EEM_INACTIVE;

    if (connect_timeout_ev)
    {
        util::evtimer_sec_add(connect_timeout_ev, 5);
    }
}

void
Eem::readCb(struct bufferevent *bev, void *arg)
{
    cout << "Usao u readCb" << endl;
    Eem *self = static_cast<Eem*>(arg);
    EemReq req;
    // eem_callback callback;
    // size_t len;
    size_t prelen;
    size_t plen;
    char *start;
    char *end;
    int *data_int = new int[2*EEM_MTU];
    static size_t count;
    int n;

    static char buf[2 * EEM_MTU];
    size_t len;
    uint8_t *data_t = new uint8_t[2*EEM_MTU];

  
    struct evbuffer *input = bufferevent_get_input(bev);
    std::vector<char> recvData(EEM_MTU);
    recvData.erase(recvData.begin(), recvData.begin() + len);

    len = evbuffer_get_length(input);
    evbuffer_copyout(input, recvData.data(),EEM_MTU -1);

    recvData.erase(recvData.begin(), recvData.begin() + count);
    cout << "Data: " << std::hex << recvData.data() << endl;

    count = count + len;
    cout << "Len:" << len << endl;
    cout << "Count:" << count << endl;
    switch (recvData[0])
                {
                    case SOH:
                        cout << "SOH///////////" << endl;
                        self->eemReq.sendACK(self->EemSocket->getBufferevent());
                        cout << "Size of vector: " << recvData.size() << endl;
                        req = self->request_queue.front();
                        if (req.pickParser(recvData.data(), recvData.size()) 
                                            == util::ErrorStatus::Success)
                        {
                            self->request_queue.erase(self->request_queue.begin(),
                                            self->request_queue.begin()+1);
                            cout << "Number of elements:" << self->request_queue.size() << endl;
                        }
                        
                        break;
                    case ACK:
                        cout << "Send Poll now!" << endl;
                        self->eemReq.sendPoll(self->EemSocket->getBufferevent());
                        
                        break;
                    case EOT:
                        cout << "End of Transmission!!!\nSend next!!" << endl;
                    
                    // default:
                        // cout << "Default" << endl;
                }

}

void
Eem::eventCb(struct bufferevent *bev, short events, void *arg)
{
    cout << "Usao u eventCb" << endl;
    Eem *self = static_cast<Eem*>(arg);
    EemReq req;
    bool noReqYet;
    if (events & BEV_EVENT_CONNECTED)
    {
        cout << "EEM connected!" << endl;
        self->eemStatus = EemState::EEM_CONNECTED;
        evtimer_del(self->connect_timeout_ev);
        req = EemReq(EemClassReq::FastSelect, 
                            SelectClassCommand::ReadBlockIdentifications);
        noReqYet = self->request_queue.empty();
        self->request_queue.push_back(req);

        if (noReqYet)
        {
            //default
            //Send neqt req!
            cout << "Empty list, send first!" << endl;
            self->sendNextReq();
        }

        return;
    }
    else if(events & BEV_EVENT_ERROR)
    {
        cout << "EEM connect failed!" << endl;

    }
    else if (events & BEV_EVENT_EOF)
    {
        printf("EOF\n");
    }
    else
    {
        printf("unknown error %d\n", events);
    }
    printf("End!\n");
}

util::ErrorStatus
Eem::write(const void *data, size_t size)
{
    return EemSocket->write(data, size);
}

util::ErrorStatus
Eem::sendNextReq()
{
    EemReq req;
    req = this->request_queue.front();
    util::ErrorStatus status;
    status = req.sendReq(this->EemSocket->getBufferevent());
    this->request_queue.erase(this->request_queue.begin());

    return status;

}


