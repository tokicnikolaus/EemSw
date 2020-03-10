#pragma once
#include "util.h"
#include <iomanip>
#include <cstring>
#include <cmath>
#include <vector>


class EemParser
{
    public:
        EemParser();
        ~EemParser();
        util::ErrorStatus parse_RN(char *, size_t);
        util::ErrorStatus parse_RB(char *, size_t);
        util::ErrorStatus parse_RI(char *, size_t);
        util::ErrorStatus parseResponse(char *, size_t);
        int parse_INT(char *buff, size_t len);
        const char *eem_getfloat(const char *, uint8_t, float **);
        float eem_atof(char *);
        const char *eem_getstr(const char *, char *, size_t);
};