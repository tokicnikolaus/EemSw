#ifndef EEM_PARSE_H
#define EEM_PARSE_H

#define EEM_NOBREAK(s) (*s && *s != '!' && *s != '*')
#define EEM_BLOCKS 16
#define EEM_STRSZ_MAX 64
#define EEM_STRSZ_FLOAT 8
#define EEM_MAX_AI 100
#define EEM_MAX_AO 100
#define EEM_MAX_DI 100
#define EEM_MAX_DO 100
#define NO_CLASS 0xFF
#define POINT   '.'

typedef enum {
    EEM_SYSTEM,
    EEM_RECTIFIER_GROUP,
    EEM_RECTIFIER,
    EEM_BATTERY_GROUP,
    EEM_BATTERY_UNIT,
    EEM_DC_DISTRIBUTION_GROUP,
    EEM_EIB_DISTRIBUTION_UNIT,
    EEM_DC_DISTRIBUTION_FUSE_UNIT,
    EEM_BATTERY_FUSE_GROUP,
    EEM_BATTERY_FUSE_UNIT,
    EEM_LVD_GROUP,
    EEM_LVD_UNIT,
    EEM_AC_GROUP,
    EEM_RECTIFIER_AC,
    EEM_OB_AC_UNIT,
    EEM_SOLAR_CONVERTER_GROUP,
    EEM_SOLAR_CONVERTER,
    EEM_SM_IO_IB2,
    EEM_UNKNOWN
} eemid_t;

struct eem_device {
    const class_index_t class_index;
    const uint8_t	ai_count;
    const uint8_t	ao_count;
    const uint8_t	di_count;
    const uint8_t	do_count;
    /* Analog Inputs */
    const char *const	ai_param[EEM_MAX_AI];
    /* Analog Outputs */
    const char *const	ao_param[EEM_MAX_AO];
    /* Digital Inputs */
    const char *const	di_param[EEM_MAX_DI];
    /* Digital Outputs */
    const char *const	do_param[EEM_MAX_DO];
};

extern const struct eem_device eem_blocks[];
extern eemid_t eem_getid(const char *);
extern const char *eem_getfloat(const char *, uint8_t, float **);
extern const char *eem_getbit(const char *, uint8_t, uint8_t **);
extern const struct eem_device *eem_device_find(const char *);

#endif
