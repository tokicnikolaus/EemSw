#include "EemReq.h"
#include <iomanip>
#include <cstring>
#include <cmath>

EemReq::EemReq()
{

}

EemReq::EemReq(EemClassReq _reqType, SelectClassCommand _selectType)
{
    requestType.req = _reqType;
    requestType.selectRequest = _selectType;
    this->prepareMessage();
}

EemReq::~EemReq()
{

}

util::ErrorStatus 
EemReq::prepareMessage()
{

    if (this->requestType.selectRequest != SelectClassCommand::NONE)
    {
        this->message = this->prepareSelect(requestType.selectRequest);
    }
    else if(this->requestType.req == EemClassReq::Poll)
    {
        // Prepare POLL
    }
    

    return util::ErrorStatus::Success;
    
}

util::ErrorStatus
EemReq::sendPoll(struct bufferevent *bev)
{
    const char ccid[] = "010000";
    std::vector<char> buffData;
    buffData.push_back(4); // EOT
    buffData.insert(buffData.end(), ccid, ccid + sizeof(ccid) - 1);
    buffData.push_back('P'); // Poll
    buffData.push_back(5); // Enq
    std::cout << "Poll prepared!" << std::endl;

    int err = bufferevent_write(bev,(void *)&buffData[0], buffData.size());
    if (err < 0)
    {
        return util::ErrorStatus::Failed;
    }
    else
    {
        return util::ErrorStatus::Success;
    }
}

util::ErrorStatus
EemReq::sendReq(struct bufferevent *bev)
{
    int err = bufferevent_write(bev,(void *)&this->message[0], this->message.size());

    if (err < 0)
    {
        return util::ErrorStatus::Failed;
    }
    else
    {
        return util::ErrorStatus::Success;
    }
}

util::ErrorStatus
EemReq::pickParser(char *buff, size_t len)
{

    switch (this->requestType.selectRequest)
    {
        case (SelectClassCommand::ReadBlock):
            this->parse_RB(buff, len);
            return util::ErrorStatus::Success;
            break;
        case (SelectClassCommand::ReadName):
            this->parse_RN(buff, len);
            return util::ErrorStatus::Success;
            break;
        case (SelectClassCommand::ReadBlockIdentifications):
            this->parse_RI(buff, len);
            // RI PARSE!
            return util::ErrorStatus::Success;
            break;
        default:
            return util::ErrorStatus::Failed;
    }

}

std::vector<char>
EemReq::prepareSelect(SelectClassCommand _selectType)
{
    std::vector<char> buffData;
    const char ccid[] = "010000"; // HARDCODED FOR NOW
    std::string req_str = getSelectType(_selectType);
    size_t checksumIndex;
    uint8_t checksum;
    buffData.push_back(EOT); // EOT
    buffData.insert(buffData.end(), ccid, ccid + sizeof(ccid) - 1);
    buffData.push_back('F');
    buffData.push_back(SOH); // Soh
    checksumIndex = buffData.size();
    buffData.insert(buffData.end(), ccid, ccid + sizeof(ccid) - 1);
    buffData.push_back(STX); // STX
    std::copy(req_str.begin(), req_str.end(), std::back_inserter(buffData));
    buffData.push_back('*'); // END
    buffData.push_back(ETX); // ETX
    checksum = getCheksum((void *)&buffData[checksumIndex], buffData.size() - checksumIndex);
    buffData.push_back(checksum);
    for (auto i = buffData.begin(); i != buffData.end(); ++i)
    std::cout << *i;

    std::cout << std::endl;
    std::cout << "Select prepared!" << std::endl;
    return buffData;
}

// int
// EemReq::callParser(char *buff, size_t len)
// {

//     (this->*parse_callback)(buff, len);

//     return 0;
// }

util::ErrorStatus 
parse(char *buff, size_t len)
{
    std::cout << "Usao" << buff << len << std::endl;
    return util::ErrorStatus::Success;
}

std::string
EemReq::getSelectType(SelectClassCommand selectType)
{
   //eem_callback cal;
    switch (selectType)
    {
        case (SelectClassCommand::ReadBlock):
            return "RB0000";
        case (SelectClassCommand::ReadName):
            return "RN";
        case (SelectClassCommand::ReadBlockIdentifications):
            return "RI";
        default:
            return "";
    }
}


util::ErrorStatus EemReq::sendACK(struct bufferevent *bev)
{

    bufferevent_write(bev, eem_ack, sizeof eem_ack);
    std::cout << "ACK sent!" << std::endl; 
    return util::ErrorStatus::Success;
}

uint8_t EemReq::getCheksum(const void *buff, size_t len)
{
    uint8_t sum = 0;
    const uint8_t *p = static_cast<const uint8_t *>(buff);
    while (len--) 
    {
        sum += *p++;
    }
    sum &= 0x7F;
    if (sum < 0x20) 
    {
        sum += 0x20;
    }
    return sum;
}
