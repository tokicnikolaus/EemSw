#include <iostream>
#include <arpa/inet.h>
#include <errno.h>
#include "baseSocket.h"
#include "baseEvent.h"
#include <stdio.h>
#include <string.h>

using namespace std;

SocketBase::SocketBase(string _server, int _port):
baseSocketConnected(socketUtil::BevStatus::Closed)
{
    sin = new sockaddr_in;
    bev = new bufferevent;
    sin->sin_family = AF_INET;
    sin->sin_port = htons(2000);

    
    if(inet_pton(AF_INET, _server.c_str(), &(sin->sin_addr)) <= 0)  
    { 
        printf("\nInvalid address/ Address not supported \n");
        throw runtime_error("SocketBase: invalid address while converting string \
                            to IP address, error:");
    }
    bev = createBuffevent();

}

struct bufferevent*
SocketBase::createBuffevent()
{
    
    struct bufferevent *tmp_bev;
    struct event_base* base = baseEvent::get_baseEvent();
    tmp_bev = bufferevent_socket_new(base, -1,
				 BEV_OPT_CLOSE_ON_FREE | BEV_OPT_CLOSE_ON_EXEC);

    if (!tmp_bev)
    {
        // Error occurred
        std::cout << "SocketBase: Error while opening new socket, error:" << endl;
        // to IMPLEMENT eem_lost !!
        // throw std::runtime_error(util::ErrHandler::buildErrorMsg("SocketBase: 
        // Error while opening new socket, error:", strerror(errno)));
        throw runtime_error("SocketBase: Error while opening new socket, error:");


    }
       
        // fd = bufferevent_getfd(bev);
    std::cout << "Success for Bev creation!" << endl;
    std::cout << "Bev: " << tmp_bev << endl;
    return tmp_bev;
    
    
}

void
SocketBase::setBuffereventNull()
{
    this->bev = NULL;
}

SocketBase::~SocketBase()
{
    try
    {
        cout << "destructor" << endl;
        delete sin;
        bufferevent_free(bev);
    }
    catch(int e)
    {
        std::runtime_error("SocketBase: Error while trying to \
            deallocate the storage associated withbufferevent");

        cout << "SocketBase: Error while deleting sin, error:" << endl;
    }
}

void
SocketBase::closeBev()
{
    if (bev)
    {
        bufferevent_free(bev);
        // bev = NULL;
        this->setBuffereventNull();
        
    }
    this->baseSocketConnected = socketUtil::BevStatus::Closed;
    
}

int
SocketBase::getFd()
{
    return this->fd;
}

void
SocketBase::ReConnectSocket(void (*_readCb)(struct bufferevent *, void *),
                            void (*_eventCb)(struct bufferevent* UNUSED,
                            short, void*), void *arg)
{
    this->closeBev();
    this->connectSocket(_readCb, _eventCb, arg);

}


util::ErrorStatus
SocketBase::connectSocket(void (*_readCb)(struct bufferevent*, void *),
                            void (*_eventCb)(struct bufferevent* UNUSED,
                            short, void*), void *arg)
{
    
    struct bufferevent *bev_tmp;
    bev_tmp = getBufferevent();
    int connect;
    struct event_base* base = baseEvent::get_baseEvent();
    const char* status;
      

    if (!bev_tmp)
    {
        puts("bufferevetn was deleted! - initialize new one!\n");
        std::cout << "bufferevetn was deleted! - initialize new one! "<< endl;
        
        this->closeBev();
        bev_tmp = this->createBuffevent();
        this->setBufferevent(bev_tmp);
        
        // if ((eemReq.initEemReq(SocketBase::getbuffevent())) 
        //      != util::ErrorStatus::Success) 
        // {
            // std::cerr << "Failed to initialized EemReq!" << std::endl;
        // }
    }
   
    connect = bufferevent_socket_connect(bev_tmp, 
                (struct sockaddr *)sin, sizeof (*sin));
   
    if(baseSocketConnected != socketUtil::BevStatus::Connected)
    {
        
        if (connect < 0)
        {
            // Failure
            std::cout << "bufferevent_socket_connect failure!" << endl;
            bufferevent_free(bev);
            status = strerror(errno);
            
            // return util::ErrorStatus::Failed;
        }
        else
        {
            std::cout << "Socket connection success!" << endl;
            baseSocketConnected = socketUtil::BevStatus::Connected;
            bufferevent_setcb(bev_tmp, _readCb, NULL, _eventCb, arg);
            if (bufferevent_enable(bev_tmp, EV_READ))
            {
                cout << "Enable Success!" << endl;
                status = strerror(errno);
                puts(status);
            }
        }
    }
    if (status)
    {
        return util::ErrorStatus::Failed;
    }
    else
        return util::ErrorStatus::Success;    

}

util::ErrorStatus
SocketBase::write(const void *data, size_t size)
{
    if(!bufferevent_write(this->getBufferevent(), data, size))
    {
        return util::ErrorStatus::Success;
    }
    else
    {
        return util::ErrorStatus::Failed;
    }
}

void
SocketBase::setBufferevent(struct bufferevent *_bev)
{
    this->bev = _bev;
}

