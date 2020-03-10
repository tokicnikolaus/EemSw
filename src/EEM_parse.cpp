#include "EEM_parse.h"
#include "EemReq.h"


EemParser::EemParser()
{}

EemParser::~EemParser()
{}

util::ErrorStatus
EemParser::parse_RB(char *buff, size_t len)
{
    char tmp[EEM_STRSZ_MAX];
    const char *s;
    float *ai_value = nullptr;
    size_t count = 14;
    if (!buff) 
    {
        printf("Null buffer!\n");
        std::cerr << "Null buff!\n" << std::endl;
        return util::ErrorStatus::Failed;
    }
    // Izmijenili jer se rusilo na drugom uvjetu -- To check!
    /*if (!buff || strncasecmp(buff, "ERR", strlen("ERR")))
    {
        std::cerr << "Timeout or error received" << std::endl;
        return util::ErrorStatus::Failed;
    }*/

    s = eem_getstr(buff, tmp, sizeof tmp); /* Get device ID */
    /* Get <Status register> from string */
    s = eem_getstr(s, tmp, sizeof tmp);
    /* Get <Analog in> from string */
    s = eem_getfloat(s, count, &ai_value);

    if (!ai_value) 
    {
        std::cerr << "Error while parsing values!" << std::endl;
        return util::ErrorStatus::Failed;
    }

    std::cout << "Parsed analog input values:" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    for (int i = 0; i < count; ++i)
    {
        std::cout << ai_value[i] << std::endl;
    }

    return util::ErrorStatus::Success;
}

util::ErrorStatus
EemParser::parse_RI(char *buff, size_t len)
{
    std::cout << "Buffer:" << buff << std::endl;

    return util::ErrorStatus::Success;
}


int
EemParser::parse_INT(char *buff, size_t len)
{
    std::cout << "Buffer:" << buff << std::endl;

    return 0;
}

util::ErrorStatus
EemParser::parse_RN(char *buff, size_t len)
{
    // eem_rn(char *buf, size_t len, void *arg)
// {
//     struct pwr *pwr = arg;
//     struct eem *e = pwr->internal;
//     size_t n;
//     if (!buf) {
// 	return;
//     }
//     eem_dump(e, buf, len);
    std::cout << "--->Name of Device1:" << buff << std::endl;
    buff[len] = '\0';
    // if (len && (n = strcspn(buff, eem_field_delim))) {
// 	if (n > sizeof e->name - 1) {
// 	    n = sizeof e->name - 1;
	// }
	// memmove(e->name, buf, n);
    std::cout << "--->Name of Device:" << buff << std::endl;
// 	e->name[n] = '\0';
//     }
// }
    // return util::ErrorStatus::Success;
    return util::ErrorStatus::Success;
}

util::ErrorStatus
EemParser::parseResponse(char *buff=NULL, size_t len=0)
{
    std::cout << "Usao u callback!!!!" << std::endl;
    char tmp[EEM_STRSZ_MAX];
    const char *s;
    float *ai_value = nullptr;
    size_t count = 14;
    if (!buff) 
    {
        printf("Null buffer!\n");
        std::cerr << "Null buff!\n" << std::endl;
        return util::ErrorStatus::Failed;
    }
    // Izmijenili jer se rusilo na drugom uvjetu -- To check!
    /*if (!buff || strncasecmp(buff, "ERR", strlen("ERR")))
    {
        std::cerr << "Timeout or error received" << std::endl;
        return util::ErrorStatus::Failed;
    }*/

    s = eem_getstr(buff, tmp, sizeof tmp); /* Get device ID */
    /* Get <Status register> from string */
    s = eem_getstr(s, tmp, sizeof tmp);
    /* Get <Analog in> from string */
    s = eem_getfloat(s, count, &ai_value);

    if (!ai_value) 
    {
        std::cerr << "Error while parsing values!" << std::endl;
        return util::ErrorStatus::Failed;
    }

    std::cout << "Parsed analog input values:" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    for (int i = 0; i < count; ++i)
    {
        std::cout << ai_value[i] << std::endl;
    }

    return util::ErrorStatus::Success;
}

const char *
EemParser::eem_getstr(const char *s, char *buf, size_t buflen)
{
    uint32_t i;

    if (!s) 
    {
        buf[0] = '\0';
        return NULL;
    }
    for (i = 0; EEM_NOBREAK(s) && i < buflen; i++) 
    {
        buf[i] = *s++;
    }
    buf[i] = '\0';
    if (*s == '!') 
    {
        s++;
    }

    return s;
}


const char*
EemParser::eem_getfloat(const char *s, uint8_t count, float **valp)
{
    char tmp[EEM_STRSZ_MAX];
    float *val;

    if (!s) 
    {
        return NULL;
    }

    if (count) 
    {
        if (!(val = *valp)) 
        {
            if (!(val = (float*)calloc(count, sizeof(float)))) 
            {
                return NULL;
            }
            *valp = val;
        }
        while (EEM_NOBREAK(s) && strlen(s) >= EEM_STRSZ_FLOAT && count--) 
        {
            strncpy(tmp, s, EEM_STRSZ_FLOAT);
            tmp[EEM_STRSZ_FLOAT] = '\0';
            *val++ = eem_atof(tmp);
            s += EEM_STRSZ_FLOAT;
        }
        while (EEM_NOBREAK(s)) 
        {
            s++;
        }
    }
    if (*s == '!') 
    {
        s++;
    }

    return s;
}


float
EemParser::eem_atof(char *s)
{
    int32_t m;
    uint32_t sign = 0;
    uint32_t e;
    union {
        uint32_t u;
        float f;
    } u;
    unsigned long ul = strtoul(s, NULL, 16);

    if (!ul) 
    {
        return 0;
    }
    if (ul == 0x7FFFFF80) 
    {
        return std::nan("");
    }

    m = (int32_t) ul;
    m >>= 7;
    m &= 0xFFFFFFFE;
    if (m < 0) 
    {
        m = -m;
        sign = 0x80000000;
    }
    m &= 0x7FFFFE;
    e = (int8_t)(ul & 0xff) + 126;
    u.u = sign | ((e << 23) & 0x7F800000) | m;

    return u.f;
}





