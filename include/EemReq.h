#pragma once
#include "util.h"
#include <event2/event.h>
#include <event2/buffer.h>
#include <event2/bufferevent.h>
#include <vector>
#include "EEM_parse.h"
#include <functional>





#define EEM_NOBREAK(s) (*s && *s != '!' && *s != '*')
#define EEM_STRSZ_FLOAT 8
#define EEM_STRSZ_MAX 64

#define SOH 1
#define STX 2
#define ETX 3
#define EOT 4
#define ENQ 5
#define ACK 6
#define NAK 0x15
#define END '*'
#define FAST_SELECT 'F'
#define POLL 'P'
#define EEM_MTU 1536
static const char eem_ack[] = {ACK};
static const char eem_delimit[] = {SOH, EOT, ACK, NAK, 0};
static const char eem_field_delim[] = "!*";




enum class EemClassReq
{
    BroadcastSelect = 'B',
    FastSelect = 'F',
    GroupSelect = 'G',
    Poll = 'P',
    TestSelect = 'T',
};

enum class SelectClassCommand
{
    ReadBlock,
    ReadBlockIdentifications,
    ReadName,
    ReadTime, // UNUSED
    ReadProductInformation, // UNUSED
    ReadListOfDevices, // UNUSED
    WriteProductInformation, // UNUSED
    WriteBlock,
    SetName, // UNUSED
    SetTime, // UNUSED
    NONE,

};

struct callReq
{
    EemClassReq req;
    SelectClassCommand selectRequest;
};



class EemReq : public EemParser
{
// protected:
public:
    EemReq();
    EemReq(EemClassReq _reqType, SelectClassCommand _selectType);
    ~EemReq();
    // friend EemParser;
    std::vector<char> message;
    callReq requestType;

    util::ErrorStatus prepareMessage();
    util::ErrorStatus sendReq(struct bufferevent *bev);
    std::vector<char> prepareSelect(SelectClassCommand _selectType);
    util::ErrorStatus sendPoll(struct bufferevent *bev);
    util::ErrorStatus pickParser(char *, size_t);
    int callParser(char *, size_t);

    util::ErrorStatus sendACK(struct bufferevent *bev);
    
    std::string getSelectType(SelectClassCommand selectType);
    void cleanBufferevent();
    uint8_t getCheksum(const void *, size_t);

};

