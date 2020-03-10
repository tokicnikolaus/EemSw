/**
 * @file eem.c
 *
 *  Routines for encoding and decoding EEM protocol
 */

#define _GNU_SOURCE
#include "config.h"
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <alloca.h>
#include <math.h>
#include <arpa/inet.h>
#include <event2/event.h>
#include <event2/buffer.h>
#include <event2/bufferevent.h>
#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>
#include "sntypes.h"
#include "sn.h"
#include "MO.h"
#include "MO-storage.h"
#include "base64.h"
#include "enl.h"
#include "util.h"
#include "system.h"
#include "alarm.h"
#include "uri.h"
#include "ctype.h"
#include "list.h"
#include "pwr.h"
#include "eem.h"
#include "eem_parse.h"
#include "PowerSystem_func.h"
#include "BatteryProfile_func.h"
#include "Threshold_func.h"
#include "loadD.h"
#include "temps.h"
#include "EEM_func.h"
#include "SNMPDevice_func.h"
#include "dc.h"
#include "thrh.h"
#include "pwr.h"
#include "snmp/snmpdev.h"

#define EEM_BAT_CHARGE_MAX  0.25
#define EEM_BAT_CHARGE_MIN  0.1
#define EEM_BAT_CHARGE_DEFAULT  0.25
#define EEM_TEMP_NOT_CONF  -273.0
#define HPM_DISCONNECT_1 "Load disconnected 1"
#define HPM_DISCONNECT_2 "Load disconnected 2"

#define DEBUG_EEM
#ifdef DEBUG_EEM
#define eem_printf(...) dep_printf_maybe(e->debug, pwr->ins, __VA_ARGS__)
#else
#define eem_printf(...) do {;} while (0)
#endif

static void eemdev_read_first(struct eem *);
static void eem_temp_sensor(struct pwr *, float, temp_type_t, size_t);
static const char *eem_connect(struct pwr *);
static void ncu_snmp_poll(struct pwr *);
static void ncu_snmp_rect_poll(struct pwr *, const oid *, size_t, uint8_t);
static void eem_lvd_block_autoconf(struct pwr *pwr, const char *id);
static const char *eem_alarm_align(MO_instance *ins, void *arg UNUSED);
static const char *eem_threshold_alarmentry_update(MO_instance *ins, void *arg UNUSED);
static void eem_create_system_thresholds(struct pwr *);

static void eemdev_read_next(struct pwr *, struct eemdev *);
void eem_set_undervolt(struct pwr *, port_t, float);
void eem_check_undervolt(struct pwr *, port_t, float);

static const char eem_ack[] = {ACK};
static const char eem_delimit[] = {SOH, EOT, ACK, NAK, 0};
static const char eem_field_delim[] = "!*";

struct eemdev {
    struct list_head list;
    struct list_head phy;
    struct list_head alarm;
    struct eem *eem;
    void *pwrp;			/* struct {pwr,pwr_in,pwr_out,pwr_bat} * */
    char id[IDLEN];
    char name[NAMELEN];
    struct eemdata data;
    eemid_t eemid;
};

struct PACKED eem_alarm {
    union {
	pwr_alarm_t type;
	pwr_in_alarm_t in_type;
	pwr_out_alarm_t out_type;
    } u;
    uint16_t bit;
};

// Index constants for EEM parameters in the Analogue Output (ao) of the EEM battery group
enum eem_battey_ao_index{
	IDX_EEM_USED_CAPACITY_LIMIT = 19,
	IDX_EEM_BATTERY_TEST_TIME   = 22,
};

#define SYSTEM_AE_PROTOCOL_EXT_ALARM_OFFSET 6

static const struct eem_alarm System_ae[] = {
    {{PWR_INTERNAL}, 0},	/* Opto Communication failure */
    {{PWR_MAINS}, 0},
    {{PWR_UNDERVOLTAGE}, 0},
    {{PWR_UNDERVOLTAGE}, 1},
    {{PWR_OVERVOLTAGE}, 0},
    {{PWR_HIGH_TEMP}, 0},
    {{PWR_REMOTE_SHUT_DOWN}, 0},
    {{PWR_DOOR}, 0},		/* Door Open */
    {{PWR_EXT}, 1},		/* BBS Battery Fuse */
    {{PWR_EXT}, 2},		/* Fan Alarm */
    {{PWR_EXT}, 3},
    {{PWR_EXT}, 4},		/* AC Grid is Off */
    {{PWR_EXT}, 5},
    {{PWR_EXT}, 6},
    {{PWR_EXT}, 7},
    {{PWR_EXT}, 8},		/* Reserved Alarm Position */
    {{PWR_EXT}, 9},
    {{PWR_EXT}, 10},
    {{PWR_EXT}, 11},
    {{PWR_EXT}, 12},
    {{PWR_EXT}, 13},
    {{PWR_EXT}, 14},
    {{PWR_EXT}, 15},
    {{PWR_INTERNAL}, 1},	/* Supervision Unit Internal Fault */
    {{PWR_INTERNAL}, 2},	/* Supervision Unit Uncalibrated */
    {{PWR_TEMP_SENSOR}, 0},
    {{PWR_LOW_TEMP}, 0},
    {{PWR_INTERNAL}, 3},	/* Outgoing Alarms Blocked */
    {{PWR_CONFIG}, 0},
    {{PWR_CAN}, 0},
    {{PWR_MULTIPLE_LOST}, 0},
    {{PWR_LOAD_SHUNT}, 0},
    {{PWR_INTERNAL}, 4},	/* Prio 2 Load Disconnected */
    {{PWR_MAINT_TIME}, 0},
    {{PWR_HIGH_LOAD}, 0},
    {{PWR_SMOKE}, 0},
    {{PWR_IGNORE}, 0},		/* Intruder alarm */
    {{PWR_IGNORE}, 0},		/* AC Mains OK */
    {{PWR_INVERTER}, 0},
    {{PWR_DC_CONV}, 0},
    {{PWR_COOLING}, 0},
    {{PWR_DIESEL}, 0},
    {{PWR_AVIATION}, 0},
    {{PWR_DC_CONV}, 1},
    {{PWR_INVERTER}, 1},
    {{PWR_HIGH_OUT_TEMP}, 0},
    {{PWR_LOW_OUT_TEMP}, 0},
    {{PWR_SPD}, 0},
    {{PWR_FAN}, 0}
};
static const struct eem_alarm Rectifier_Group_ae[] = {
    {{PWR_IGNORE}, 0},
    {{PWR_OUT_MAINS}, 0},
    {{PWR_IGNORE}, 0},		/* Mains Fault */
    {{PWR_OUT_LOST}, 3},
    {{PWR_IGNORE}, 0},		/* ECO mode activated */
    {{PWR_IGNORE}, 0},
    {{PWR_IGNORE}, 0},
    {{PWR_IGNORE}, 0},
    {{PWR_IGNORE}, 0},
    {{PWR_IGNORE}, 0},
    {{PWR_IGNORE}, 0},
    {{PWR_IGNORE}, 0},
    {{PWR_OUT_FAIL}, 4}
};
static const struct eem_alarm Rectifier_ae[] = {
    {{PWR_IGNORE}, 0},		/* Rectifier on/off */
    {{PWR_IN_FAIL}, 0},
    {{PWR_IN_MAINS}, 0},
    {{PWR_IN_OVERVOLTAGE}, 0},
    {{PWR_IN_HIGH_TEMP}, 0},
    {{PWR_IGNORE}, 0},		/* Rectifier Current Limit */
    {{PWR_IN_MAINS}, 0},	/* Rectifier AC Fault */
    {{PWR_IN_LOST}, 0},
    {{PWR_IN_FAIL}, 1},
    {{PWR_IN_LIMIT}, 0},	/* Power Limit (ShortCirq. Protection) */
    {{PWR_IN_FAN}, 0}
};
static const struct eem_alarm Battery_Group_ae[] = {
    {{PWR_IGNORE}, 0},		/* Battery Test Running */
    {{PWR_BAT_TEST}, 0},	/* Battery Test Failure */
    {{PWR_IGNORE}, 0},		/* Battery Boost Charge */
    {{PWR_BAT_BAD}, 0},		/* Bad Battery */
    {{PWR_IGNORE}, 0},		/* Manual Battery Test Running */
    {{PWR_IGNORE}, 0},		/* Schedule Battery Test Running */
    {{PWR_IGNORE}, 0},		/* Automatic Battery Test running */
    {{PWR_IGNORE}, 0},		/* Manual Battery Boost Charge */
    {{PWR_IGNORE}, 0},		/* Cyclic Battery Boost Charge running */
    {{PWR_IGNORE}, 0},		/* Automatic Battery Boost Charge running */
    {{PWR_IGNORE}, 0},		/* Battery Current Limitation running */
    {{PWR_BAT_TEMP_SENSOR}, 0},
    {{PWR_BAT_HIGH_TEMP}, 0},
    {{PWR_BAT_HIGH_TEMP}, 1},	/* Very High Temperature */
    {{PWR_BAT_LOW_TEMP}, 0},
    {{PWR_BAT_SHUNT}, 0},
    {{PWR_BAT_CURRENT}, 0},
    {{PWR_IGNORE}, 0},		/* Equalizing Battery Boost Charge running */
    {{PWR_BAT_LOAD}, 0}
};
static const struct eem_alarm Battery_Unit_ae[] = {
    {{PWR_IGNORE}, 0},		/* Temperature Sensor Fault */
    {{PWR_IGNORE}, 0},		/* High Temperature */
    {{PWR_IGNORE}, 0},		/* Very High Temperature */
    {{PWR_IGNORE}, 0}		/* Low Temperature */
};
static const struct eem_alarm DC_Distribution_Fuse_Unit_ae[] = {
    {{PWR_FUSE}, 0},		/* Fuse 1 */
    {{PWR_FUSE}, 1},
    {{PWR_FUSE}, 2},
    {{PWR_FUSE}, 3},
    {{PWR_FUSE}, 4},
    {{PWR_FUSE}, 5},
    {{PWR_FUSE}, 6},
    {{PWR_FUSE}, 7},
    {{PWR_FUSE}, 8},
    {{PWR_FUSE}, 9}
};
static const struct eem_alarm Battery_Fuse_Unit_ae[] = {
    {{PWR_BAT_FUSE}, 0},	/* Fuse Failure 1 */
    {{PWR_BAT_FUSE}, 1},	/* Fuse Failure 2 */
    {{PWR_BAT_FUSE}, 2},	/* Fuse Failure 3 */
    {{PWR_IGNORE}, 0},		/* Uncalibrated */
    {{PWR_BAT_FUSE}, 3}		/* Fuse Failure 4 */
};
static const struct eem_alarm LVD_Group_ae[] = {
    {{PWR_IGNORE}, 0}		/* Load Disconnect Error */
};
static const struct eem_alarm LVD_Unit_ae[] = {
    {{PWR_OUT_DISCONNECT}, 0}	/* LVD Disconnected */
};
static const struct eem_alarm Rectifier_AC_ae[] = {
    {{PWR_INTERNAL}, 5},	/* Unit Uncalibrated */
    {{PWR_INTERNAL}, 6},	/* Supervision Fail */
    {{PWR_IGNORE}, 0},		/* Over voltage, Phase 1-2 */
    {{PWR_IGNORE}, 0},		/* Severe Over voltage, Phase 1-2 */
    {{PWR_IGNORE}, 0},		/* Under voltage, Phase 1-2 */
    {{PWR_IGNORE}, 0},		/* Severe Under voltage, Phase 1-2 */
    {{PWR_IGNORE}, 0},		/* Over voltage, Phase 1-3 */
    {{PWR_IGNORE}, 0},		/* Severe Over voltage, Phase 1-3 */
    {{PWR_IGNORE}, 0},		/* Under voltage, Phase 1-3 */
    {{PWR_IGNORE}, 0},		/* Severe Under voltage, Phase 1-3 */
    {{PWR_IGNORE}, 0},		/* Over voltage, Phase 2-3 */
    {{PWR_IGNORE}, 0},		/* Severe Over voltage, Phase 2-3 */
    {{PWR_IGNORE}, 0},		/* Under voltage, Phase 2-3 */
    {{PWR_IGNORE}, 0},		/* Severe Under voltage, Phase 2-3 */
    {{PWR_IGNORE}, 0},		/* Over voltage, Phase 1 */
    {{PWR_IGNORE}, 0},		/* Severe Over voltage, Phase 1 */
    {{PWR_IGNORE}, 0},		/* Under voltage, Phase 1 */
    {{PWR_IGNORE}, 0},		/* Severe Under voltage, Phase 1 */
    {{PWR_IGNORE}, 0},		/* Over voltage, Phase 2 */
    {{PWR_IGNORE}, 0},		/* Severe Over voltage, Phase 2 */
    {{PWR_IGNORE}, 0},		/* Under voltage, Phase 2 */
    {{PWR_IGNORE}, 0},		/* Severe Under voltage, Phase 2 */
    {{PWR_IGNORE}, 0},		/* Over voltage, Phase 3 */
    {{PWR_IGNORE}, 0},		/* Severe Over voltage, Phase 3 */
    {{PWR_IGNORE}, 0},		/* Under voltage, Phase 3 */
    {{PWR_IGNORE}, 0},		/* Severe Under voltage, Phase 3 */
    {{PWR_IN_MAINS}, 1},	/* Mains Failure */
    {{PWR_IN_MAINS}, 2},	/* Severe Mains Failure */
    {{PWR_IGNORE}, 0},		/* High Current, Phase 1 */
    {{PWR_IGNORE}, 0},		/* High Current, Phase 2 */
    {{PWR_IGNORE}, 0},		/* High Current, Phase 3 */
    {{PWR_IGNORE}, 0},		/* High Frequency Failure */
    {{PWR_IGNORE}, 0},		/* High Temperature Failure */
    {{PWR_IGNORE}, 0},		/* Low Frequency Failure */
    {{PWR_IGNORE}, 0}		/* Low Temperature Failure */
};
static const struct eem_alarm Solar_Converter_Group_ae[] = {
    {{PWR_IGNORE}, 0},		/* Double converter failure */
    {{PWR_IGNORE}, 0},		/* Reserved */
    {{PWR_IGNORE}, 0},		/* Converter Lost */
    {{PWR_IGNORE}, 0},		/* Capacity Limitation */
    {{PWR_OUT_UNBALANCED}, 0},	/* Unbalanced current */
    {{PWR_OUT_OVERVOLTAGE}, 0},	/* Overvoltage */
    {{PWR_OUT_LOST}, 0}		/* Local communication failure */
};
static const struct eem_alarm Solar_Converter_ae[] = {
    {{PWR_IGNORE}, 0},		/* Converter on/off */
    {{PWR_IN_FAIL}, 0},		/* Converter failure */
    {{PWR_IGNORE}, 0},		/* Mains Failure */
    {{PWR_IGNORE}, 0},		/* Over voltage */
    {{PWR_IGNORE}, 0},		/* Reserved */
    {{PWR_IGNORE}, 0},		/* Converter Current Limit */
    {{PWR_IGNORE}, 0},		/* Reserved */
    {{PWR_IN_LOST}, 0},		/* Converter Communication Fail */
    {{PWR_IGNORE}, 0},		/* Reserved */
    {{PWR_IGNORE}, 0},		/* Power Limit */
    {{PWR_IN_FAN}, 0},		/* Fan Failure */
    {{PWR_IN_LIMIT}, 0},	/* Limited due to high temp */
    {{PWR_IGNORE}, 0},		/* Fans in full speed */
    {{PWR_IGNORE}, 0},		/* WALK in */
    {{PWR_IGNORE}, 0},		/* Equalized charging */
    {{PWR_IGNORE}, 0},		/* Test in process */
    {{PWR_HIGH_TEMP}, 0},	/* Over temperature */
    {{PWR_IGNORE}, 0}		/* Unbalanced current */
};
static const struct eem_alarm SM_IO_IB2_ae[] = {
    {{PWR_EXT}, 1},
    {{PWR_EXT}, 2},
    {{PWR_EXT}, 3},
    {{PWR_EXT}, 4},
    {{PWR_EXT}, 5},
    {{PWR_EXT}, 6},
    {{PWR_EXT}, 7},
    {{PWR_EXT}, 8},
};
#undef a
static const struct eem_ae {
    const struct eem_alarm *ae;
    size_t count;
} eem_ae[] = {
    { System_ae, MAXCOUNT(System_ae) },
    { Rectifier_Group_ae, MAXCOUNT(Rectifier_Group_ae) },
    { Rectifier_ae, MAXCOUNT(Rectifier_ae) },
    { Battery_Group_ae, MAXCOUNT(Battery_Group_ae) },
    { Battery_Unit_ae, MAXCOUNT(Battery_Unit_ae) },
    { NULL, 0 },
    { NULL, 0 },
    { DC_Distribution_Fuse_Unit_ae, MAXCOUNT(DC_Distribution_Fuse_Unit_ae) },
    { NULL, 0 },
    { Battery_Fuse_Unit_ae, MAXCOUNT(Battery_Fuse_Unit_ae) },
    { LVD_Group_ae, MAXCOUNT(LVD_Group_ae) },
    { LVD_Unit_ae, MAXCOUNT(LVD_Unit_ae) },
    { NULL, 0 },
    { Rectifier_AC_ae, MAXCOUNT(Rectifier_AC_ae) },
    { Rectifier_AC_ae, MAXCOUNT(Rectifier_AC_ae) },
    { Solar_Converter_Group_ae, MAXCOUNT(Solar_Converter_Group_ae) },
    { Solar_Converter_ae, MAXCOUNT(Solar_Converter_ae) },
    { SM_IO_IB2_ae, MAXCOUNT(SM_IO_IB2_ae) },
};

static const perceivedSeverity_t eem_severity[] = {
    critical,
    major,
    minor,
    warning
};

#define pwr_type_ncu(pwr) (pwr->type == PWR_NCU)
#define OID(n) n, OID_LENGTH(n)
#define ld_id(x) ((e->ld[x]) && (e->ld[x]->instanceId[0]))

static const struct eem_alarm ncu_remapping_DC_Distribution_Fuse_Unit_ae[] = {
    {{PWR_COOLING}, 0},                 //Input DI1 PWR_DOOR
    {{PWR_DOOR}, 0},                    //Input DI2
    {{PWR_BATTERY_DISCONNECT}, 2},      //Input DI3
    {{PWR_FUSE}, 0},                    //Input DI4
    {{PWR_IGNORE}, 0},   //Input DI5
    {{PWR_IGNORE}, 0},   //Input DI6
    {{PWR_IGNORE}, 0},   //Input DI7
    {{PWR_IGNORE}, 0},   //Input DI8
    {{PWR_IGNORE}, 0},   //Input DI9
    {{PWR_IGNORE}, 0}    //Input DI10
};

static struct alarm_defSpecificProblem ncu_alarm_defSpecificProblem_list[] = {
        { PowerSystem_index, "COOLING", "Climate Fault" },
        { PowerSystem_index, "BATTERY_DISCONNECT", "Battery Disconnect" },
        { PowerSystem_index, "BAT_HIGH_TEMP", "High Battery Temperature" },
        { PowerSystem_index, "BAT_TEST", "Battery Test Fault" },
        { PowerSystem_index, "BAT_LOW_TEMP", "Low Battery Temperature" },
        { PowerSystem_index, "OVERVOLTAGE", "Overvoltage DC" },
        { PowerSystem_index, "UNDERVOLTAGE", "Undervoltage DC" },
        { Rectifier_index, "LOST", "Communication Fault" },
        { Rectifier_index, "FAN", "Fan Fault" },
        { Rectifier_index, "HIGH_TEMP", "High Temperature" },
        { Rectifier_index, "OVERVOLTAGE", "Overvoltage AC" },
        { Rectifier_index, "FAIL", "Rectifier Fault" },
};

static struct pwr_alarm_defSpecificProblem_list pwr_alarm_defSpecificProblem_list[] ={
        {PWR_NCU, ncu_alarm_defSpecificProblem_list, MAXCOUNT(ncu_alarm_defSpecificProblem_list)},
};

static struct thr_allowed_alentry_list ncu_thr_allowed_alentry_list[] = {
        {PowerSystem_index,"TempAlarmHigh",{{0}}},
        {PowerSystem_index,"TempAlarmLow",{{0}}},
        {PowerSystem_index,"HighSystemVoltage",{{0}}},
        {TemperatureSensor_index,"BattTempHigh", {"OUT_OF_RANGE", {0}}},
        {TemperatureSensor_index,"BattTempLow", {"OUT_OF_RANGE", {0}}},
        {Battery_index,"TempDisconnectHigh",{{0}}},
        {Battery_index,"TempReconnectHigh",{{0}}},
};


/*
 * SNMP OIDs
 */
static const oid identModel_oid[] = { 1,3,6,1,4,1,6302,2,1,1,2,0 };
static const oid identCtrlFWVersion_oid[] = { 1,3,6,1,4,1,6302,2,1,1,3,0 };
static const oid identName_oid[] = { 1,3,6,1,4,1,6302,2,1,1,4,0 };
static const oid identSerialNum_oid[] = { 1,3,6,1,4,1,6302,2,1,1,6,0 };
static const oid rectKey_oid[] = { 1,3,6,1,4,1,6302,2,1,2,11,4,1,1 };
static const oid rectProdNum_oid[] = { 1,3,6,1,4,1,6302,2,1,2,11,4,1,2,0 };
static const oid rectHWVersion_oid[] = { 1,3,6,1,4,1,6302,2,1,2,11,4,1,3,0 };
static const oid rectSWVersion_oid[] = { 1,3,6,1,4,1,6302,2,1,2,11,4,1,4,0 };
static const oid rectSerialNum_oid[] = { 1,3,6,1,4,1,6302,2,1,2,11,4,1,5,0 };
static const oid rectIdent_oid[] = { 1,3,6,1,4,1,6302,2,1,2,11,4,1,7,0 };
static oid rectNumInstalled_oid[] = { 1,3,6,1,4,1,6302,2,1,2,11,1,0 };
static oid rectKeyI_oid[] = { 1,3,6,1,4,1,6302,2,1,2,11,4,1,1,0 };
uint8_t *ncu_rect_snmp_id = NULL;

static struct _snmpget_multi_req {
    oid *oidp;
    size_t oidlen;
    snmpget_t type;
} *ncu_snmp_multi_req = NULL;

/**
 * @brief Calculate EEM checksum for a message buffer
 * @param buf	Message buffer
 * @param len	Message buffer length
 * @return	checksum
 */
static ushort
eem_checksum(const void *buf, size_t len)
{
    uint8_t sum = 0;
    const uint8_t *p = buf;
    while (len--) {
	sum += *p++;
    }
    sum &= 0x7F;
    if (sum < 0x20) {
	sum += 0x20;
    }
    return sum;
}

/**
 * @brief Free and clear an EEM bus request
 * @param er
 */
static void
eemr_free(struct eemr *er)
{
    list_del(&er->list);
    free(er);
}

static struct eemr *
eemr_first(struct eem *e)
{
    struct list_head *head = &e->queue;
    if (list_empty(head)) {
	return NULL;
    }
    return list_entry(head->next, struct eemr, list);
}

static void
eem_select(struct eem *e)
{
    struct eemr *er;
    char buf[EEM_MTU];
    char *p;
    char *bcc_start;
    uint8_t bcc;
    size_t len;
    if (!(er = eemr_first(e))) {
	return;
    }
    p = buf;
    *p++ = EOT;
    *p++ = e->cc_id[0];
    *p++ = e->cc_id[1];
    *p++ = '0';
    *p++ = '0';
    *p++ = '0';
    *p++ = '0';
    *p++ = FAST_SELECT;
    *p++ = SOH;
    bcc_start = p;
    *p++ = e->cc_id[0];
    *p++ = e->cc_id[1];
    *p++ = '0';
    *p++ = '0';
    *p++ = '0';
    *p++ = '0';
    *p++ = STX;
    memmove(p, er->request, er->request_len);
    p += er->request_len;
    *p++ = END;
    *p++ = ETX;
    bcc = eem_checksum(bcc_start, p - bcc_start);
    len = p - buf;
    buf[len++] = bcc;
    bufferevent_write(e->bev, buf, len);
    er->send_count++;
    evtimer_sec_add(e->timeout_event, EEM_TIMEOUT);
}

static void
eem_poll(struct eem *e)
{
    char buf[POLL_LEN];
    char *p = buf;
    *p++ = EOT;
    *p++ = e->cc_id[0];
    *p++ = e->cc_id[1];
    *p++ = '0';
    *p++ = '0';
    *p++ = '0';
    *p++ = '0';
    *p++ = POLL;
    *p = ENQ;
    bufferevent_write(e->bev, buf, sizeof buf);
    evtimer_sec_add(e->timeout_event, EEM_TIMEOUT);
}

#ifdef EEM_FAKE
const struct fake {
    char *request;
    char *response;
} fake[] = {
    {"RN", "ACU+!01!$*"},
//  {"RI", "00000!02000!02011!02021!03000!03011!04000!04011!05000!05011!07000!07011!07022!09000!09011!09021*"},
    {"RP0000", "00000!Power System*"},
    {"DL0000", "0000!0001!0002!006E!006F*"},
    {"RP0200", "02000!Rectifier Group*"},
    {"DL0200", "0200!0002!0003*"},
    {"RP0201", "02011!Rectifier1*"},
    {"DL0201", "0201!0002*"},
    {"RP0202", "02021!Rectifier2*"},
    {"DL0202", "0202!0003*"},
    {"RP0300", "03000!Battery Group*"},
    {"DL0300", "0300!006E*"},
    {"RP0301", "03011!Battery Main Cabinet*"},
    {"DL0301", "0301!006E*"},
    {"RP0400", "04000!DC Distribution*"},
    {"DL0400", "0400*"},
    {"RP0401", "04011!DC Main Cabinet*"},
    {"DL0401", "0401*"},
    {"RP0500", "05000!Battery Fuse Group*"},
    {"DL0500", "0500*"},
    {"RP0501", "05011!Battery Fuse Main Cabinet*"},
    {"DL0501", "0501*"},
    {"RP0700", "07000!LVD Group*"},
    {"DL0700", "0700*"},
    {"RP0701", "07011!LVD Main Cabinet*"},
    {"DL0701", "0701*"},
    {"RP0702", "07022!LVD Main Cabinet*"},
    {"DL0702", "0702*"},
    {"RP0900", "09000!AC Group*"},
    {"DL0900", "0900!0001*"},
    {"RP0901", "09011!Rectifier AC*"},
    {"DL0901", "0901!0001*"},
    {"RP0902", "09021!AC Unit*"},
    {"DL0902", "0902*"},
    {"DP0001", "0001!#0100#020#031R482000      #0401090200208#05A02#069.01*"},
    {"DP0002", "0002!#0102#021#031R482000      #0401090200061#05A02#061.02*"},
    {"DP0003", "0003!#0102#021#031R482000      #0401090200208#05A02#069.01*"},
    {"DP006E", "006E!#0100#020#03M820B#0403110700289#05A04#062.23*"},
};
#endif

static void
eemr_send_next(struct eem *e)
{
    struct eemr *er;
    eem_callback_t *callback;
    if (!e->bev) {
	return;
    }
    while ((er = eemr_first(e))) {
	if (er->request_len) {
#ifdef EEM_FAKE
	    size_t i;
	    char buf[EEM_MTU];
	    if ((callback = er->user_callback)) {
		for (i = 0; i < MAXCOUNT(fake); i++) {
		    if (!strcmp(er->request, fake[i].request)) {
			COPY(buf, fake[i].response);
			callback(buf, strlen(buf), er->user_data);
			eemr_free(er);
			break;
		    }
		}
		continue;
	    }
#endif
	    eem_select(e);
	    return;
	}
	/* Dummy request used for queue checkpoint callback */
	if ((callback = er->user_callback)) {
	    callback(NULL, 0, er->user_data);
	}
	eemr_free(er);
    }
}

static void
eem_close(struct eem *e)
{
    if (e->bev) {
	bufferevent_free(e->bev);
	e->bev = NULL;
    }
    e->state = EEM_INACTIVE;
    if (e->event) {
	evtimer_sec_add(e->event, 5);
    }
}

static void
eemdev_datafree(struct eemdev *ed)
{
    if (ed->data.ai_value) {
	free(ed->data.ai_value);
	ed->data.ai_value = NULL;
    }
    if (ed->data.ao_value) {
	free(ed->data.ao_value);
	ed->data.ao_value = NULL;
    }
    if (ed->data.di_value) {
	free(ed->data.di_value);
	ed->data.di_value = NULL;
    }
    if (ed->data.do_value) {
	free(ed->data.do_value);
	ed->data.do_value = NULL;
    }
}

static void
eem_forget(struct eem *e)
{
    struct eemdev *ed;
    list_for_each_entry (ed, &e->device, list) {
	eemdev_datafree(ed);
    }
    e->rb_rq_loop = false;
    pwr_forget(e->pwr);
}

static void
eem_lost(struct eem *e)
{
    eem_close(e);
    eem_forget(e);
    pwr_alarm(e->pwr, PWR_LOST, 0, indeterminate, true);
}

static void
eemr_timeout(int fd UNUSED, short what UNUSED, void *arg)
{
    struct pwr *pwr = arg;
    struct eem *e = pwr->internal;
    struct eemr *er;
    eem_callback_t *callback;
    time_t t, td;
    if (!(er = eemr_first(e))) {
	return;
    }
    if (!e->bev) {
	eem_connect(pwr);
	return;
    }
    if (er->send_count < MAX_SEND_COUNT) {
	eem_select(e);
	return;
    }
    t = time_monotonic();
    if (!pwr->last_time.tv_sec) {
	pwr->last_time.tv_sec = t;
    } else {
	td = t - pwr->last_time.tv_sec;
	if (td > EEM_LOST_TIMEOUT) {
	    eem_lost(e);
	}
    }
    eem_printf("timeout\n");
    if ((callback = er->user_callback)) {
	callback(NULL, 0, er->user_data);
    }
    eemr_free(er);
    eemr_send_next(e);
}

/**
 * @brief Create a new EEM request and add it to the send queue
 * @param ei		EEM bus interface
 * @param request	Request data
 * @param request_len	Length of request data
 * @param callback	Callback function pointer
 * @param arg		Request user data
 * @return		The EEM request data structure
 */
static struct eemr *
eemr_new(struct pwr *pwr, const char *request,
	 eem_callback_t *callback, void *arg)
{
    struct eem *e = pwr->internal;
    size_t request_len;
    struct eemr *er = NULL;
    int empty;
    if (e && (er = MALLOCP(struct eemr))) {
	ZERO(er);
	if (request) {
	    request_len = strlen(request);
	    if (request_len > sizeof(er->request)) {
		request_len = sizeof(er->request);
	    }
	    memmove(er->request, request, request_len);
	} else {
	    request_len = 0;
	    er->request[0] = '\0';
	}
	er->request_len = request_len;
	er->user_callback = callback;
	er->user_data = arg;
	empty = list_empty(&e->queue);
	list_add_tail(&er->list, &e->queue);
	if (empty) {
	    switch (e->state) {
	    case EEM_INACTIVE:
		eem_connect(pwr);
		break;
	    case EEM_CONNECTING:
		break;
	    default:
		eemr_send_next(e);
		break;
	    }
	}
    }
    return er;
}

static void
eem_readcb(struct bufferevent *bev, void *arg)
{
    struct pwr *pwr = arg;
    struct eem *e = pwr->internal;
    static size_t count;
    static char buf[2 * EEM_MTU];
    eem_callback_t *callback;
    struct eemr *er;
    uint8_t bcc;
    size_t len;
    size_t prelen;
    size_t plen;
    char *start;
    char *end;
    if (!e || !MO_getref(&pwr->ins)) {
	return;
    }
    while ((len = bufferevent_read(bev, buf + count, sizeof buf - count))) {
	if (len == (size_t)-1) {
	    break;
	}
	count += len;
	while (count) {
	    buf[count] = '\0';
	    prelen = strcspn(buf, eem_delimit);
	    if (prelen < count) {
		if (prelen) {
		    dump("EEM?>", buf, prelen);
		}
		start = buf + prelen;
		switch (start[0]) {
		case SOH:
		    len = count - prelen;
		    if (!(end = memchr(start, ETX, len))) {
			goto more;
		    }
		    plen = end - start + 1;
		    if (plen > len) {
			goto more;
		    }
		    bcc = eem_checksum(start + 1, plen - 1);
		    if (bcc != start[plen]) {
			eem_printf("Incorrect checksum %02X != %02X\n",
				   start[plen], bcc);
		    }
		    bufferevent_write(e->bev, eem_ack, sizeof eem_ack);
		    plen++;
		    if ((er = eemr_first(e))) {
			if ((callback = er->user_callback)) {
			    callback(start + 8, plen - 10, er->user_data);
			}
			eemr_free(er);
		    }
		    count = len - plen;
		    if (count) {
			memmove(buf, end + 1, count);
		    }
		    e->tmout_cnt = 0;
		    break;
		case ACK:
		    if (e && !list_empty(&e->queue) && e->bev) {
			eem_poll(e);
		    }
		    goto skip;
		case NAK:
		    eem_printf("NAK\n");
		    goto skip;
		case EOT:
		    eemr_send_next(e);
		skip:
		    if (--count) {
			memmove(start, start + 1, count);
		    }
		    if(++e->tmout_cnt >= 5) {
		        eem_printf("timeout [%d]\n", start[0], e->tmout_cnt);
		        if(!(e->tmout_cnt % 5)) {
		            eem_lost(e);
		            eem_printf("t disconnect\n", start[0], e->tmout_cnt);
		            if(evtimer_pending(e->event, NULL)) {
		                evtimer_del(e->event);
		                evtimer_sec_add(e->event, 15);
		            }
		            return;
		        }
		    }
		    break;
		}
	    } else {
		dump("EEM?>", buf, count);
		count = 0;
		break;
	    }
	}
    more:;
    }
}

static void
eemr_free_all(struct eem *e)
{
    struct list_head *head = &e->queue;
    struct list_head *pos;
    struct list_head *next;
    struct eemr *er;
    list_for_each_safe (pos, next, head) {
	er = list_entry(pos, struct eemr, list);
	eemr_free(er);
    }
}

static void
eemdev_free(struct eemdev *ed)
{
#ifdef EEMPHY
    struct eemphr *eh;
    struct eemphr *next_eh;
    list_for_each_entry_safe (eh, next_eh, &ed->phy, list) {
	free(eh);
    }
#endif
    eemdev_datafree(ed);
    list_del(&ed->list);
    free(ed);
}

static void
eem_free(struct eem *e)
{
    struct eemdev *ed;
    struct eemdev *next_ed;
#ifdef EEMPHY
    struct eemphy *ep;
    struct eemphy *next_ep;
#endif
    if (e->timeout_event) {
	event_free(e->timeout_event);
    }
    if (e->scan_event) {
	event_free(e->scan_event);
    }
    if (e->event) {
	event_free(e->event);
	e->event = NULL;
    }
    if (e->snmp_event) {
        event_free(e->snmp_event);
    }
    list_for_each_entry_safe (ed, next_ed, &e->device, list) {
	eemdev_free(ed);
    }
    eem_close(e);
#ifdef EEMPHY
    list_for_each_entry_safe (ep, next_ep, &e->physical, list) {
	list_del(&ep->list);
	free(ep);
    }
#endif
    eemr_free_all(e);
    free(e);
}

#ifdef DEBUG_EEM
static void
eem_dump(struct eem *e, char *buf, size_t len)
{
    struct pwr *pwr = e->pwr;
    struct eemr *er;
    char *p = NULL;
    size_t n = 0;
    if ((er = eemr_first(e))) {
	p = er->request;
	n = er->request_len;
    }
    while (n > 128) {
	eem_printf("%.*s\\\n", 128, p);
	n -= 128;
	p += 128;
    }
    eem_printf("%.*s*\n", n, p);
    while (len > 128) {
	eem_printf("%.*s\\\n", 128, buf);
	len -= 128;
	buf += 128;
    }
    eem_printf("%.*s\n", len, buf);
}
#else
#define eem_dump(e, buf, len) do {;} while (0)
#endif

static size_t
eem_fieldlen(const char *p)
{
    return strcspn(p, eem_field_delim);
}

static int
eemdev_cmp(struct eemdev *ed, const char *id)
{
    return memcmp(ed->id, id, sizeof ed->id - 1);
}

struct eemdev *
eemdev_find(struct eem *e, const char *id)
{
    struct eemdev *ed;
    list_for_each_entry (ed, &e->device, list) {
	if (!eemdev_cmp(ed, id)) {
	    return ed;
	}
    }
    return NULL;
}

static struct eemdev *
eemdev_new(struct eem *e, char *id, bool addFirst)
{
    struct eemdev *ed;
    if (!(ed = eemdev_find(e, id))) {
	if (!(ed = MALLOCP(struct eemdev))) {
	    return NULL;
	}

	ZERO(ed);
	INIT_LIST_HEAD(&ed->phy);
	INIT_LIST_HEAD(&ed->alarm);
	ed->eem = e;
	COPY(ed->id, id);
	ed->eemid = eem_getid(id);
	if (addFirst) {
	    list_add(&ed->list, &e->device);
	} else {
	    list_add_tail(&ed->list, &e->device);
	}
    }
    return ed;
}

#ifdef EEMPHY
static struct eemphy *
eemphy_new(struct eem *e, char *id)
{
    struct eemphy *ep = MALLOCP(struct eemphy);
    if (ep) {
    ZERO(ep);
    ep->eem = e;
    COPY(ep->id, id);
    list_add_tail(&ep->list, &e->physical);
    }
    return ep;
}

static int
eemphy_cmp(struct eemphy *ep, char *id)
{
    return memcmp(ep->id, id, sizeof ep->id - 1);
}

static struct eemphy *
eemphy_find(struct eem *e, char *id)
{
    struct eemphy *ep;
    list_for_each_entry (ep, &e->physical, list) {
    if (!eemphy_cmp(ep, id)) {
        return ep;
    }
    }
    return NULL;
}

static struct eemphr *
eemphr_new(void)
{
    struct eemphr *eh = MALLOCP(struct eemphr);
    if (eh) {
    ZERO(eh);
    }
    return eh;
}

static struct eemphr *
eemphr_find(struct eemdev *ed, char *id)
{
    struct eemphr *eh;
    list_for_each_entry (eh, &ed->phy, list) {
    if (!eemphy_cmp(eh->eemphy, id)) {
        return eh;
    }
    }
    return NULL;
}

static size_t
pidcpy(char *dest, const char *src, size_t maxlen)
{
    size_t len = strcspn(src, "#*");
    size_t n = len;
    if (n >= maxlen) {
    n = maxlen - 1;
    }
    memmove(dest, src, n);
    dest[n] = '\0';
    return len;
}

#define s(field, src) pidcpy(ep->field, src, sizeof ep->field)

static void
eem_dp(char *buf, size_t len, void *arg)
{
    struct eemphy *ep = arg;
    size_t n;
    char *p;
    if (!buf) {
    return;
    }
    eem_dump(ep->eem, buf, len);
    buf[len] = '\0';
    if ((n = eem_fieldlen(buf)) < sizeof ep->id - 1
    || eemphy_cmp(ep, buf)) {
    return;
    }
    n++;
    len -= n;
    p = buf + n;
    while (len > 2 && p[0] == '#' && p[1] == '0') {
    p += 3;
    switch (p[-1]) {
    case '1':
        n = s(group, p);
        break;
    case '2':
        n = s(subgroup, p);
        break;
    case '3':
        n = s(product_number, p);
        break;
    case '4':
        n = s(serial_number, p);
        break;
    case '5':
        n = s(HW_revision, p);
        break;
    case '6':
        n = s(SW_revision, p);
        break;
    }
    len -= n;
    p += n;
    n = strspn(p, eem_field_delim);
    p += n;
    len -= n;
    }
}

#undef s

static void
eem_dl(char *buf, size_t len, void *arg)
{
    struct eemdev *ed = arg;
    struct eem *e = ed->eem;
    struct eemphy *ep;
    struct eemphr *eh;
    char command[7];
    size_t n;
    char *p;
    if (!buf) {
    return;
    }
    eem_dump(e, buf, len);
    buf[len] = '\0';
    if ((n = eem_fieldlen(buf)) < sizeof ed->id - 1
    || eemdev_cmp(ed, buf)) {
    return;
    }
    n++;
    len -= n;
    p = buf + n;
    while (len && (n = eem_fieldlen(p))) {
    if (n < sizeof ep->id - 1) {
        break;
    }
    if (!(ep = eemphy_find(e, p))) {
        if (!(ep = eemphy_new(e, p))) {
        break;
        }
        snprintf(command, sizeof command, "DP%.4s", p);
        eemr_new(e->pwr, command, eem_dp, ep);
    }
    if (!eemphr_find(ed, p)
        && (eh = eemphr_new())) {
        eh->eemphy = ep;
        list_add_tail(&eh->list, &ed->phy);
    }
    len -= n;
    p += n;
    n = strspn(p, eem_field_delim);
    p += n;
    len -= n;
    }
}
#endif

static void
eem_rp(char *buf, size_t len UNUSED, void *arg)
{
    struct eemdev *ed = arg;
    size_t n;
    char *p;
    if (!buf || prefix(buf, "ERR")) { /* Timeout or error */
	return;
    }
    eem_dump(ed->eem, buf, len);
    if ((n = eem_fieldlen(buf)) >= sizeof ed->id - 1
	|| eemdev_cmp(ed, buf)) {
	p = buf + n + 1;
	if ((n = eem_fieldlen(p))) {
	    if (n >= sizeof ed->name) {
		n = sizeof ed->name - 1;
	    }
	    memmove(ed->name, p, n);
	    ed->name[n] = '\0';
	}
    }
}

static char *
eem_lvd_name(struct pwr *pwr, uint32_t id)
{
    switch (pwr->type) {
    case PWR_NCU:
        switch (id) {
        case 1:
        case 2:
            return ssprintf("Load%d",id);
        default:
            return ssprintf("LVD%d", id);
        }
        break;
    default:
        return ssprintf("LVD%d", id);
    }
}

static void
eem_get_done(char *buf UNUSED, size_t len UNUSED, void *arg)
{
    struct pwr *pwr = arg;
    struct eem *e = pwr->internal;
    const struct eem_device *device;
    struct eemdev *ed;
#ifdef EEMPHY
    struct eemphy *ep;
    struct eemphr *eh;
    MO_instance *eqparent;
#endif
    MO_instance *ins, *parent, *inbat;
    struct Equipment eq, *ep;
    struct SDN *sp;
    uint8_t class_index = NO_CLASS;
    struct pwr_in *in;

    if (!(parent = MO_getref(&pwr->ins))) {
	return;
    }
    ZERO(&eq);
    COPY(eq.productName, e->name);
    if ((ins = MO_SDN())
	&& (sp = MO_tdata(false, ins))) {
	COPY(eq.connectedTo, sp->depIPInterface);
	COPY(eq.serialNumber, "1");
    }
    COPY(eq.identifier, ip_ssaddr_get(&e->dest));
    // Make sure we have a Rectifiers before any Rectifier is created
    list_for_each_entry (ed, &e->device, list) {
        if ((device = eem_device_find(ed->id))) {
            if (device->class_index == Rectifiers_index) {
		break;
	    } else if (device->class_index == Rectifier_index) {
		eemdev_new(e, "0200", true);
		break;
	    }
        }
    }
    list_for_each_entry (ed, &e->device, list) {
	if ((device = eem_device_find(ed->id))) {
	    class_index = device->class_index;
	}
	if (!strncmp(ed->id, CSU_ID, strlen(CSU_ID))) {
	    class_index = Equipment_index;
	}

	switch (class_index) {
	case Rectifier_index:
	    ed->pwrp = in = pwr_rectifier_new(pwr, ed->id);
	    if (pwr_type_ncu(pwr) && in && in->ins) {
	        eem_printf("%s - rect[%s] SN %s.", __func__, in->id, in->SerialNum);
	        if ((ins = MO_addAnotherEquipment(in->ins, in->SerialNum))) {
	            in->ineq = ins;
	            if(ins->class_index == Equipment_index
	                    && (ep = MO_data(ins))) {
	                COPY(ep->productNumber, in->productNum);
	                COPY(ep->swProductRevision, in->swProdRevision);
	            }
	        }
	    }
	    break;
	case Rectifiers_index:
	    ed->pwrp = pwr_rectifiers_new(pwr, ed->id);
	    break;
	case SolarConverter_index:
	    ed->pwrp = pwr_solar_new(pwr, ed->id);
	    break;
	case SolarConverters_index:
	    ed->pwrp = pwr_out_solar_new(pwr, ed->id);
	    break;
	case LoadDisconnect_index:
        eem_lvd_block_autoconf(pwr, ed->id);
        if ((inbat = MO_getref(&pwr->inbat))) {
            BatteryProfile_add(inbat, EEM_SYSTEM_BATTERY_PROFILE_ID);
        }
        break;
	default:
	    if (class_index != NO_CLASS) {
	        if (pwr_type_ncu(pwr))
	            ins = MO_addAnotherEquipment(parent, e->SerialNum);
	        else
	            ins = MO_create(parent, class_index, ed->id);
	    }
	    break;
	}
    if (!eq.productName[0]) {
        COPY(eq.productName, ed->name);
    }
    if (!strncmp(ed->id, CSU_ID, strlen(CSU_ID))) {
        COPY(eq.swProductRevision, e->SWrevision);
        COPY(eq.serialNumber, e->SerialNum);
        if(!strcmp(e->ProductModel, "NCU")) {
            COPY(eq.productNumber, "M830B");
        }
    }
#ifdef EEMPHY
	eqparent = ins;
	list_for_each_entry (eh, &ed->phy, list) {
	    ep = eh->eemphy;
	    eem_printf("%s: %s %s %s %s %s %s\n",
		       ep->id, ep->group, ep->subgroup, ep->product_number,
		       ep->serial_number, ep->HW_revision, ep->SW_revision);
	    COPY(eq.productName, ed->name);
	    COPY(eq.productNumber, ep->product_number);
	    COPY(eq.serialNumber, ep->serial_number);
	    COPY(eq.productRevision, ep->HW_revision);
	    COPY(eq.swProductRevision, ep->SW_revision);
	    snprintf(eq.identifier, sizeof eq.identifier, "%s:%s:%s",
		     ep->id, ep->group, ep->subgroup);
	    if (class_index == Equipment_index) {
		eqparent = ins;
	    } else if (!(ins = MO_create(eqparent, Equipment_index, ep->id))) {
		break;
	    }
	    MO_setData(ins, &eq);
	    ZERO(&eq);
	}
	if (eqparent && eq.productName[0]) {
	    MO_setData(eqparent, &eq);
	    ZERO(&eq);
	}
#else
	if (class_index == Equipment_index
	        && (!pwr_type_ncu(pwr)
	            || !strncmp(ed->id, CSU_ID, strlen(ed->id)))) {
	    MO_setData(ins, &eq);
	}
	ZERO(&eq);
#endif
    }
    MO_traverse(false, &parent, eem_alarm_align, pwr);
    MO_traverse(false, &parent, eem_threshold_alarmentry_update, pwr);
    if (config_modified()) {
	config_delayed_write(100);
    }
    eemdev_read_first(e);
}

static void
eem_ri(char *buf, size_t len, void *arg)
{
    struct pwr *pwr = arg;
    struct eem *e = pwr->internal;
    struct eemdev *ed;
    char command[7];
    size_t n;
    char *p;
    const struct eem_device *device;
    if (!buf) {
	goto out;
    }
    eem_dump(e, buf, len);
    buf[len] = '\0';
    p = buf;
    while (len && (n = eem_fieldlen(p))) {
	if (n >= sizeof ed->id - 1) {
	    if ((device = eem_device_find(p))
		&& (device->class_index == Rectifiers_index)) {
		// Makes sure Rectifiers is before any Rectifier in the list.
		if (!(ed = eemdev_new(e, p, true))) {
		    break;
		}
	    } else {
		if (!(ed = eemdev_new(e, p, false))) {
		    break;
		}
	    }
        snprintf(command, sizeof command, "RP%.4s", p);
	    if (!ed->name[0]) {
		eemr_new(pwr, command, eem_rp, ed);
	    }
#ifdef EEMPHY
	    if (memcmp(p, CSU_ID, sizeof ed->id - 1)) {
		command[0] = 'D';
		command[1] = 'L';
		eemr_new(pwr, command, eem_dl, ed);
	    }
#endif
	}
	len -= n;
	p += n;
	n = strspn(p, eem_field_delim);
	p += n;
	len -= n;
    }
    if ((ed = eemdev_new(e, CSU_ID, false))) {
	COPY(ed->name, e->name);
    }
    if (pwr_type_ncu(pwr)) {
        evtimer_sec_add(e->snmp_event, 1);
    } else
        eemr_new(pwr, NULL, eem_get_done, pwr);
out:
    if (e->scan_event) {
	evtimer_sec_add(e->scan_event, EEM_SCAN_PERIOD);
    }
}

static void
eem_rn(char *buf, size_t len, void *arg)
{
    struct pwr *pwr = arg;
    struct eem *e = pwr->internal;
    size_t n;
    if (!buf) {
	return;
    }
    eem_dump(e, buf, len);
    buf[len] = '\0';
    if (len && (n = eem_fieldlen(buf))) {
	if (n > sizeof e->name - 1) {
	    n = sizeof e->name - 1;
	}
	memmove(e->name, buf, n);
	e->name[n] = '\0';
    }
}

static void
eem_error(struct bufferevent *bev UNUSED, short event, void *arg)
{
    struct pwr *pwr = arg;
    struct eem *e = pwr->internal;
    if (event & BEV_EVENT_CONNECTED) {
	eem_printf("Connected\n");
	evtimer_del(e->event);
	e->state = EEM_CONNECTED;
	eemr_free_all(e);
	e->rb_rq_loop = false;
	if (!e->name[0]) {
	    eemr_new(pwr, "RN", eem_rn, pwr);
	}
	eemr_new(pwr, "RI", eem_ri, pwr);
	pwr_alarm(pwr, PWR_LOST, 0, indeterminate, false);
	return;
    } else if (event & BEV_EVENT_EOF) {
	eem_printf("EOF\n");
    } else if (event & BEV_EVENT_ERROR) {
	eem_printf("Connect failed\n");
    } else {
	eem_printf("unknown error %d\n", event);
    }
    eem_lost(e);
}

static const char *
eem_connect(struct pwr *pwr)
{
    struct eem *e = pwr->internal;
    const char *status;
    struct bufferevent *bev;
    ip_ssaddr_t *ss = &pwr->ip.addr_port;
    if (!e) {
	return "";
    }
    bev = bufferevent_socket_new(event_base, -1,
				 BEV_OPT_CLOSE_ON_FREE | BEV_OPT_CLOSE_ON_EXEC);
    if (!bev) {
	goto err;
    }
    if (bufferevent_socket_connect(bev, SOCKADDR(ss), sizeof *ss)) {
	bufferevent_free(bev);
	goto err;
    }
    bufferevent_setcb(bev, eem_readcb, NULL, eem_error, pwr);
    bufferevent_enable(bev, EV_READ);
    e->bev = bev;
    e->state = EEM_CONNECTING;
    return NULL;
err:
    status = strerror(errno);
    eem_printf("%s: %s\n", __func__, status);
    eem_lost(e);
    return status;
}

static void
eem_connect_timeout(int fd UNUSED, short what UNUSED, void *arg)
{
    struct pwr *pwr = arg;
    struct eem *e = pwr->internal;
    if (e->state == EEM_INACTIVE) {
        e->rb_rq_loop = false;
        eem_connect(pwr);
    }
}

void
eem_reconnect(struct eem *e)
{
    eem_close(e);
    eem_connect(e->pwr);
}

const char *
eem_getstr(const char *s, char *buf, size_t buflen)
{
    uint32_t i;

    if (!s) {
	buf[0] = '\0';
	return NULL;
    }
    for (i = 0; EEM_NOBREAK(s) && i < buflen; i++) {
	buf[i] = *s++;
    }
    buf[i] = '\0';
    if (*s == '!') {
	s++;
    }
    return s;
}

static void
eemdev_read_system(struct pwr *pwr, struct eemdev *ed)
{
	float *ai, *ao, tmax = NAN;
	float temp[NCU_NUM_BATT_TEMP_SENS] = {NAN, NAN, NAN};
	float amb_temp;
	uint8_t i;

	if ((ai = ed->data.ai_value)) {
		pwr->enabled = true;
		pwr->voltage = ai[0];
		pwr->current = ai[1];
		pwr->power = ai[2] * 1000; /* conversion from kW to W*/
		if (pwr_type_ncu(pwr)) {
		    amb_temp = ai[13];   // ambient
		} else {
		    amb_temp = ai[6];    // ambient
		    // outside - ai[7]
		}
		eem_temp_sensor(pwr, amb_temp, AMBIENT, 0);
		if (pwr_type_ncu(pwr)) {
		    for (i = 0; i < NCU_NUM_BATT_TEMP_SENS; i++) {
		        temp[i] = ai[10 + i];   // battery
		        eem_temp_sensor(pwr, temp[i], BATT, i);
		        if (!isnan(temp[i]) && ((isnan(tmax)) || (temp[i] > tmax))) {
		            tmax = temp[i];
		        }
		    }
		    pwr_bat_temp3(pwr, temp[0], temp[1], temp[2]);
		    pwr->bat_temperature = tmax;
		}
	}
	if (pwr->enabled && (ao = ed->data.ao_value)) {
	    pwr->device_config.f[bat_float_charge_voltage] = ao[0];
	    pwr->device_config.f[bat_low_voltage_minor] = ao[1];
	    pwr->device_config.f[bat_low_voltage_major] = ao[3];
	    pwr->device_config.f[dc_volt_max] = ao[5];
	    if (pwr_type_ncu(pwr)) {
	        pwr->device_config.f[batt1_high_temp] = ao[14];
	        pwr->device_config.f[batt1_low_temp] = ao[15];
	        pwr->device_config.f[batt2_high_temp] = ao[16];
	        pwr->device_config.f[batt2_low_temp] = ao[17];
	        pwr->device_config.f[batt3_high_temp] = ao[18];
	        pwr->device_config.f[batt3_low_temp] = ao[19];
	        pwr->device_config.f[bat_high_env_temp] = ao[20];
	        pwr->device_config.f[bat_low_env_temp] = ao[21];
	    } else {
	        pwr->device_config.f[bat_high_env_temp] = ao[9];
	        pwr->device_config.f[bat_low_env_temp] = ao[10];
	    }
	}
	eem_create_system_thresholds(pwr);
}

static int
eem_faulty_rectifiers(struct eem *e)
{
    int count = 0;
    struct eemdev *ed;
    uint8_t *di;
    if (e) {
	list_for_each_entry (ed, &e->device, list) {
	    if (!strncmp(ed->id, "02", 2)
		    && strncmp(ed->id + 2, "00", 2)
		    && (di = ed->data.di_value)
		    && di[2]) {
		count++;
	    }
	}
    }
    return count;
}

static const char *
eem_rectifier_count(MO_instance *instance, void *arg)
{
    int *countp = arg;
    if (instance->class_index == Rectifier_index) {
        (*countp)++;
    }
    return NULL;
}


static void
eem_rectifiers_alarms(struct pwr *pwr, struct eemdev *ed, bool mains_alarm)
{
    struct eem *e = ed->eem;
    int fail_count = 0, lost_count = 0;
    bool ov_alarm = false, ht_alarm = false;
    bool limit_alarm = false, fan_alarm = false;
    struct eemdev *dev;
    uint8_t *di;
    float *ai = NULL;
    int mo_count = 0;
    bool is_rect_acin_nan = false;

    if (pwr->ins)
    MO_traverseContainment(false, pwr->ins, NULL, eem_rectifier_count, &mo_count);
    lost_count = mo_count;
    if (e) {
	list_for_each_entry (dev, &e->device, list) {
	    if (!strncmp(dev->id, "02", 2)
		    && strncmp(dev->id + 2, "00", 2)
		    && (di = dev->data.di_value)) {
		if (di[2] | di[16]) {
		    fail_count++;
		}
		if (di[4] | di[12]) {
		    mains_alarm = true;
		}
		if (di[6]) {
		    ov_alarm = true;
		}
		if (di[8]) {
		    ht_alarm = true;
		}
		is_rect_acin_nan = (ai = dev->data.ai_value) && isnanf(ai[4]);
		if (!(di[14] || // not Rectifier Communication Fail
		  (is_rect_acin_nan && !di[0]))){ //Input AC Voltage NaN and rectifier in slot
		    lost_count--;
		}
		if (di[18]) {
		    limit_alarm = true;
		}
		if (di[20]) {
		    fan_alarm = true;
		}
	    }
	}

	// FAIL Alarm Entry
	pwr_rectifiers_fail_alarm(pwr, ed->pwrp, pwr->in_count, fail_count);
	// LOST Alarm Entry
	pwr_rectifiers_lost_alarm(pwr, ed->pwrp, mo_count, lost_count);
	// Other alarms
	pwr_out_alarm(pwr, ed->pwrp, PWR_OUT_MAINS, 1, warning, mains_alarm);
	pwr_out_alarm(pwr, ed->pwrp, PWR_OUT_OVERVOLTAGE, 1, major, ov_alarm);
	pwr_out_alarm(pwr, ed->pwrp, PWR_OUT_HIGH_TEMP, 0, minor, ht_alarm);
	pwr_out_alarm(pwr, ed->pwrp, PWR_OUT_LIMIT, 0, warning, limit_alarm);
	pwr_out_alarm(pwr, ed->pwrp, PWR_OUT_FAN, 0, minor, fan_alarm);
    }
}

static void
eemdev_read_rectifier_group(struct pwr *pwr, struct eemdev *ed)
{
    struct pwr_out *out;
    float *ai, *ao;

    if (!(out = ed->pwrp)) {
	if (!(out = pwr_out_new(pwr, ed->id))) {
	    return;
	}
	ed->pwrp = out;
    }
    if ((ai = ed->data.ai_value)) {
	out->voltage = ai[0];
	out->current = ai[1];
	out->power = NAN;
	pwr->in_count = ai[5];
	out->enabled = true;
    }
    if ((ao = ed->data.ao_value)) {
	pwr->input_current_limit = ao[1];
    }
    pwr->fail_count = eem_faulty_rectifiers(ed->eem);
}

static void
eemdev_read_rectifier(struct pwr *pwr, struct eemdev *ed)
{
    struct pwr_in *in;
    uint8_t *di;
    float *ai;
    if (!(in = ed->pwrp)) {
	if (!(in = pwr_in_new(pwr, ed->id))) {
	    return;
	}
	ed->pwrp = in;
    }
    if ((ai = ed->data.ai_value) &&
            (di = ed->data.di_value)) {
	in->active = (!(di[4])      // Rectifier Mains failure
	        && !di[14]);        // Communication fail
	in->output.voltage = ai[0];
	in->output.current = ai[1];
	in->output.power = NAN;
	in->temperature = ai[2];
	in->utilization = ai[3];
	in->input.voltage = ai[4];
	in->run_time = lrintf(3600 * ai[5]);
	if(in->active) {
	    in->last_seen = time(NULL);
	}
    }
}

static void
eemdev_read_battery_group(struct pwr *pwr, struct eemdev *ed)
{
    struct pwr_bat *bat;
    float *ai, *ao;
    uint8_t *di;
    bool test_running = false;

    if (!(bat = ed->pwrp)) {
	if (!(bat = pwr_bat_new(pwr, "1"))) {
	    return;
	}
	ed->pwrp = bat;
    }
    if (!pwr_type_ncu(pwr) && (ai = ed->data.ai_value)) {
	pwr->bat_temperature = ai[2];
    }
    if ((ao = ed->data.ao_value) && pwr) {
	pwr->device_config.f[bat_charge_curr_max] = ao[7];
	pwr->device_config.f[bat_boost_charge_voltage] = ao[8];
	pwr->device_config.f[bat_nominal_capacity] = 3600 * ao[18];
    }
    if ((di = ed->data.di_value)) {
        bat->boost_charge = (di[14] | di[16] | di[18]);
        test_running = (di[8] | di[10] | di[12]);
    }
    pwr_mode_set(pwr, (test_running ? PWR_MODE_TEST :
            (bat->boost_charge ? PWR_MODE_BOOST : PWR_MODE_FLOAT)));
}

static void
eemdev_read_battery(struct pwr *pwr, struct eemdev *ed)
{
    struct pwr_bat *bat;
    struct bat *batt;
    bool batt_alm_active = false, curr_neg;
    float *ai, *ao, temp;
    struct alarm *ap;
    float curr_resolution = 0;

    if (!(bat = ed->pwrp)) {
	if (!(bat = pwr_bat_new(pwr, "1"))) {
	    return;
	}
	ed->pwrp = bat;
    }
    if ((ai = ed->data.ai_value)) {
	pwr->bat_voltage = ai[0];
	if(!isnanf(pwr->bat_current) &&
	        !isnanf(ai[1])) {
	    if ((batt = pwr_bat_get(pwr))) {
	        curr_resolution = batt->current_resolution;
	    }
	    curr_neg = (ai[1] < -curr_resolution);
	    if((ap = find_alarm_from_ins(pwr->ins)) &&
	            ap->eventType == equipmentAlarm &&
	            !strcmp(ap->specificProblem, "Battery Discharging")) {
	        batt_alm_active = true;
	    }
	    if(curr_neg != batt_alm_active) {
	        pwr_alarm(pwr, PWR_BAT_DISCHARGE, 0, warning, curr_neg);
	    }
	}
	pwr->bat_current = bat->current = ai[1];
    } else {
	pwr->bat_voltage = NAN;
	pwr->bat_current = NAN;
	pwr->bat_temperature = NAN;
    }
    if ((ao = ed->data.ao_value) && ao[0]) {
        pwr->device_config.f[bat_nominal_capacity] = 3600 * ao[0];
    }
    if (!pwr_type_ncu(pwr)) {
        temp = pwr->bat_temperature;
        eem_temp_sensor(pwr, temp, BATT, 0);
    }
}

static void
eemdev_read_lvd_group(struct pwr *pwr, struct eemdev *ed)
{
    float *ao;
    if (pwr) {
        if ((ao = ed->data.ao_value)) {
            pwr->device_config.f[bat_high_temp_major]=ao[8];
            pwr->device_config.f[bat_high_temp_minor]=ao[9];
        }
    }
}

static void
eemdev_read_lvd_unit(struct pwr *pwr, struct eemdev *ed)
{
    float *ao;
    uint8_t *d, *di;
    struct eem *e;
    struct loadD *ld;
    uint32_t cont_id = 0;

    if (!pwr || !(e = pwr->internal)) {
        return;
    }

    if (isctype(ed->id[3], DIGIT)) {
        cont_id = ed->id[3] - '0';
    }
    if (!IS_IN_RANGE(cont_id, 1, 3)) {
        return;
    }

    if ((ao = ed->data.ao_value)) {
        switch (cont_id) {
        case MAIN_LD_CONID:
            pwr->device_config.f[dc_load_disconnect_time1] = ao[0];
            pwr->device_config.f[dc_load_disconnect_volt1] = ao[1];
            pwr->device_config.f[dc_load_reconnect_volt1] = ao[3];
            break;
        case PRIO_LD_CONID:
            pwr->device_config.f[dc_load_disconnect_time2] = ao[0];
            pwr->device_config.f[dc_load_disconnect_volt2] = ao[1];
            pwr->device_config.f[dc_load_reconnect_volt2] = ao[3];
            break;
        case BLVD_LD_CONID:
            // Time currently not in use
            pwr->device_config.f[bat_disconnect_time] = ao[0];
            pwr->device_config.f[bat_disconnect_voltage] = ao[1];
            pwr->device_config.f[bat_reconnect_voltage] = ao[3];
            break;
        }
    }

    if (cont_id > 2 || !(ld = IS_LLVD(cont_id - 1))) {
        return;
    }
    if ((di = ed->data.di_value)) {
        if (di[0]) {
            ld->cont_state = ContactorOFF;
        } else {
            ld->cont_state = ContactorON;
        }
    }
    if ((d = ed->data.do_value)) {
        if (d[0]) {
            ld->llvd_en = true;
        } else {
            ld->llvd_en = false;
        }
        if (!d[2]) {
            ld->dm = BatteryVoltage;
        } else {
            ld->dm = Time;
        }
    }
}

static int
eem_solar_count(struct pwr *pwr)
{
    struct eem *e = pwr->internal;
    struct eemdev *ed;
    int count = 0;
    list_for_each_entry (ed, &e->device, list) {
	if (!strncmp(ed->id, SOLAR_PREFIX, sizeof SOLAR_PREFIX)
	    && strncmp(ed->id, SOLAR_GROUP_ID, sizeof SOLAR_GROUP_ID)) {
	    count++;
	}
    }
    return count;
}

static const char *
eem_solar_lost(MO_instance *ins, void *arg)
{
    struct pwr *pwr = arg;
    if (ins->class_index == SolarConverter_index) {
	if (!eemdev_find(pwr->internal, ins->instanceId)) {
	    MO_alarm(LICENSE_SOLAR, ins, LOST, true);
	}
    }
    return NULL;
}

static void
eemdev_read_solar_converter_group(struct pwr *pwr, struct eemdev *ed)
{
    struct pwr_out *out;
    float *ai;
    uint8_t *di;
    if (!(out = ed->pwrp)) {
	if (!(out = pwr_out_new(pwr, ed->id))) {
	    return;
	}
	ed->pwrp = out;
    }
    if ((ai = ed->data.ai_value)) {
	out->voltage = ai[0];
	out->current = ai[1];
	out->power = NAN;
	if (eem_solar_count(pwr) != ai[5]) {
	    MO_traverse(false, NULL, eem_solar_lost, pwr);
	}
    }
    if ((di = ed->data.di_value)) {
        out->enabled = !(di[12]);
    }
}

static void
eemdev_read_solar_converter(struct pwr *pwr, struct eemdev *ed)
{
    struct pwr_in *in;
    MO_instance *ins;
    float *ai;
    uint8_t *di;
    if (!(in = ed->pwrp)) {
	if (!(in = pwr_in_new(pwr, ed->id))) {
	    return;
	}
	ed->pwrp = in;
    }
    if ((ai = ed->data.ai_value)) {
	in->output.voltage = ai[0];
	in->output.current = ai[1];
	in->output.power = NAN;
	in->run_time = lrintf(3600 * ai[5]);
	in->input.voltage = ai[7];
	in->input.current = ai[8];
	in->temperature = ai[9];
	in->utilization = ai[10];
	if (in->output.current > 0.1
	    && !in->alarm[PWR_IN_LOST].active
	    && (ins = MO_getref(&in->ins))) {
	    MO_alarm(LICENSE_SOLAR, ins, LOST, false);
	}
    }
    if ((di = ed->data.di_value)) {
	in->active = !(di[0]);
	in->failed = di[2];
	in->comm_fail = di[14];
    }
}

static void
eemdev_read_dcdistribution_group(struct pwr *pwr, struct eemdev *ed)
{
    uint8_t i;
    struct dc* dc = NULL;
    float *ai;
    MO_instance* dcm_ins = NULL;
    MO_instance* parent = MO_getref(&pwr->ins);
    char *id;
    if (!pwr_type_ncu(pwr)
            || !ed)
        return;
    if ((ai = ed->data.ai_value)) {
        for (i = 0; i < NCU_NUM_CONS_DCM; i++) {
            if(i <= 1) {
                id = ssprintf("Load%d", i + 1);
            }
            else if(i == 2){
                id = "LoadB";
            }
            else { /*not defined*/ }
            if ((dcm_ins = MO_findChild(parent, DCMeter_index, id))
                    && (dc = MO_private(dcm_ins))) {
                dc->voltage = ai[0];
            }
        }
    }
}

static void
eemdev_read_eib_distribution_unit(struct pwr *pwr, struct eemdev *ed)
{
    float *ai;
    int i;
    MO_instance* parent = MO_getref(&pwr->ins);
    MO_instance* dcm_ins = NULL;
    struct dc* dc = NULL;
    char *id;
    if (!pwr_type_ncu(pwr)
            || !ed)
        return;
    for (i = 0; i < NCU_NUM_CONS_DCM; i++) {
        if(i <= 1) {
            id = ssprintf("Load%d", i + 1);
        }
        else if(i == 2){
            id = "LoadB";
        } else { /* not defined */ }
        if (!(dcm_ins = MO_findChild(parent, DCMeter_index, id))
                && !(dcm_ins = pwr_dcmeter_new(pwr, id))) {
            continue;
        }
        if ((dc = MO_private(dcm_ins))) {
            if ((ai = ed->data.ai_value)) {
                dc->current = ai[i];
            } else {
                dc->current = NAN;
            }
        }
    }
}

typedef void eemdev_read_callback_t(struct pwr *, struct eemdev *);

eemdev_read_callback_t *eemdev_read_callback[] = {
    eemdev_read_system,
    eemdev_read_rectifier_group,
    eemdev_read_rectifier,
    eemdev_read_battery_group,
    eemdev_read_battery,
    eemdev_read_dcdistribution_group,
    eemdev_read_eib_distribution_unit,
    NULL,			/* DC Distribution Fuse Unit */
    NULL,			/* Battery Fuse Group */
    NULL,			/* Battery Fuse Unit */
    eemdev_read_lvd_group,	/* LVD Group */
    eemdev_read_lvd_unit,	/* LVD Unit */
    NULL,			/* AC Group */
    NULL,			/* Rectifier AC */
    NULL,			/* Ob AC Unit */
    eemdev_read_solar_converter_group,
    eemdev_read_solar_converter,
    NULL,           /* SM IO IB2 */
};

static void
eem_rb(char *buf, size_t len, void *arg)
{
    struct eemdev *ed = arg;
    struct eem *e;
    struct pwr *pwr;
    const struct eem_device *device;
    eemdev_read_callback_t *callback;
    char tmp[EEM_STRSZ_MAX];
    const char *s;
    eemid_t id;
    uint16_t i;
    float *ai;
    float *ao;
    uint8_t *d;
    e = ed->eem;
    pwr = e->pwr;

    if (!buf || prefix(buf, "ERR")) { /* Timeout or error */
	eemdev_datafree(ed);
	goto next;
    }
    eem_dump(e, buf, len);
    s = eem_getstr(buf, tmp, sizeof tmp); /* Get device ID */
    if (strlen(tmp) < IDLEN - 1
	|| eemdev_cmp(ed, tmp)) {
	eem_printf("%s: Reply from wrong device\n", __func__);
	goto next;
    }
    id = eem_getid(tmp);
    if (id == EEM_UNKNOWN) {
	goto next;
    }
    device = &eem_blocks[id];
    /* Get <Status register> from string (UNUSED) */
    s = eem_getstr(s, tmp, sizeof tmp);
    /* Get <Analog in> from string */
    s = eem_getfloat(s, device->ai_count, &ed->data.ai_value);
    /* Get <Analog out> from string */
    s = eem_getfloat(s, device->ao_count, &ed->data.ao_value);
    /* Get <Digital in> from string */
    s = eem_getbit(s, device->di_count, &ed->data.di_value);
    /* Get <Digital out> from string */
    eem_getbit(s, device->do_count, &ed->data.do_value);
    if ((ai = ed->data.ai_value)) {
	eem_printf("Analog Inputs (%d):\n", device->ai_count);
	for (i = 0; i < device->ai_count; i++) {
	    eem_printf("-[%d] %s: %g\n", i, device->ai_param[i], ai[i]);
	}
    }
    if ((ao = ed->data.ao_value)) {
	eem_printf("Analog Outputs (%d):\n", device->ao_count);
	for (i = 0; i < device->ao_count; i++) {
	    eem_printf("-[%d] %s: %g\n", i, device->ao_param[i], ao[i]);
	}
    }
    if ((d = ed->data.di_value)) {
	eem_printf("Digital Inputs (%d):\n", device->di_count);
	for (i = 0; i < device->di_count; i++) {
	    eem_printf("-[%d] %s: %s\n", i, device->di_param[i],
		       d[i] ? "true" : "false");
	}
    }
    if ((d = ed->data.do_value)) {
	eem_printf("Digital Outputs (%d):\n", device->do_count);
	for (i = 0; i < device->do_count; i++) {
	    eem_printf("-[%d] %s: %s\n", i, device->do_param[i],
		       d[i] ? "true" : "false");
	}
    }
    if ((callback = eemdev_read_callback[id])) {
	callback(pwr, ed);
    }
next:
    eemdev_read_next(pwr, ed);
}

static void
eemdev_read(struct eemdev *ed)
{
    struct pwr *pwr = ed->eem->pwr;
    char command[7];
    struct eem *e = ed->eem;
    if(!e->rb_rq_loop)
        e->rb_rq_loop = true;
    snprintf(command, sizeof command, "RB%.4s", ed->id);
    eemr_new(pwr, command, eem_rb, ed);
}

static void
eemdev_read_first(struct eem *e)
{
    struct list_head *head = &e->device;
    struct pwr *pwr;

    if (!e
            || !(pwr = e->pwr)
            || list_empty(head)) {
        return;
    }
    if (e->rb_rq_loop) {
        eem_printf("RB request loop already active");
        return;
    }
    eemdev_read(list_entry(head->next, struct eemdev, list));
}

static void eem_read_alarms(struct pwr *, int);

static bool eem_bat_current_negative(struct pwr *pwr)
{
    if(!isnanf(pwr->bat_current) && pwr->bat_current < 0.)
        return true;
    return false;
}

static void
ncu_snmp_done(void *arg, bool success)
{
    struct pwr *pwr = arg;
    struct eem *e;
    struct snmpget_multi *sm;
    char *s, idx[5];
    size_t i, req_cnt;
    struct pwr_in *in = NULL;
    uint8_t rect_id, param;

    if (!pwr
            || !(e = pwr->internal)
            || !(sm = e->sm)
            || !success) {
        return;
    }
    req_cnt = NCUSYS_NUMSNMPREQ + NCURECT_NUMSNMPREQ * e->pwr->in_count;
    for (i = 0; i < req_cnt; i++) {
        if (!(s = snmpget_multi_string(sm, i)))
            goto done;
        str_remove_trail_ws(s);
        switch (i) {
        case NCUSYS_SNMPMODEL:
            COPY(e->ProductModel, s);
            break;
        case NCUSYS_SNMPFWVER:
            COPY(e->SWrevision, s);
            break;
        case NCUSYS_SNMPNAME:
            break;
        case NCUSYS_SNMPSERNM:
            COPY(e->SerialNum, s);
            break;
        default:
            rect_id = (i - NCUSYS_NUMSNMPREQ) / NCURECT_NUMSNMPREQ;
            param = (i - NCUSYS_NUMSNMPREQ) % NCURECT_NUMSNMPREQ;
            switch(param) {
            case NCURECT_SNMPPROD:
                snprintf(idx, sizeof(idx), "02%02d", rect_id + 1);
                if ((in = pwr_in_find(pwr, idx))
                        && in->ineq) {
                    COPY(in->productNum, s);
                }
                break;
            case NCURECT_SNMPSWVER:
                if (in)
                    COPY(in->swProdRevision, s);
                break;
            case NCURECT_SNMPSERNUM:
                if (in) {
                    COPY(in->SerialNum, s);
                    in = NULL;
                }
                break;
            }
        }
        free(s);
done:
        free((void *)ncu_snmp_multi_req[i].oidp);
    }
    if (ncu_snmp_multi_req) {
        free(ncu_snmp_multi_req);
        ncu_snmp_multi_req = NULL;
    }
    if (ncu_rect_snmp_id) {
        free(ncu_rect_snmp_id);
        ncu_rect_snmp_id = NULL;
    }
    if (e->sm) {
        free(e->sm);
    }
    eem_get_done(NULL, 0, pwr);
}

static void
ncu_snmp_rect_idx(void *arg, const oid *oidp, size_t oidlen,
        const void *data, size_t len, snmpget_t snmptype,
        const char *status)
{
    struct pwr *pwr = arg;
    struct eem *e;
    struct snmp_device *sd;
    int32_t val;
    size_t idx;
    static uint8_t id = 0;

    if (!pwr
            || !(e = pwr->internal)
            || !(sd = pwr->snmp_device)) {
        eem_get_done(NULL, 0, pwr);
        return;
    }
    if (status) {
        pwr_printf("%s: %s\n", __FUNCTION__, status);
        eem_get_done(NULL, 0, pwr);
        return;
    } else if (oid_prefix(oidp, oidlen, OID(rectNumInstalled_oid))
            && snmptype == SNMPGET_INT32) {
        val = snmpget_int32(data, len);
        if (val > 0
                && !ncu_rect_snmp_id) {
            ncu_rect_snmp_id = malloc((size_t)val * sizeof(uint8_t));
            e->snmp_status = NCUSNMP_RECTKEY;
        } else
            e->snmp_status = NCUSNMP_MULTIRQ;
    } else if (oid_prefix(oidp, oidlen, OID(rectKey_oid))
           && snmptype == SNMPGET_INT32) {
        val = snmpget_int32(data, len);
        idx = MAXCOUNT(rectKeyI_oid) - 1;
        rectKeyI_oid[idx] = val;
        if (ncu_rect_snmp_id) {
            ncu_rect_snmp_id[id] = (uint8_t)val;
            id++;
            e->snmp_status = NCUSNMP_RECTID;
        } else {
            e->snmp_status = NCUSNMP_MULTIRQ;
        }
    } else if (e->snmp_status == NCUSNMP_RECTID) {  // done
        e->snmp_status = NCUSNMP_MULTIRQ;
        id = 0;
    } else {
        e->snmp_status = NCUSNMP_INVALID;
        pwr_printf("%s: SNMP error", __FUNCTION__);
    }
    evtimer_sec_add(e->snmp_event, 1);
}

static void
ncu_add_snmp_request(const oid *oid1, size_t len, size_t *id)
{
    size_t slen = len * sizeof(oid);
    oid *oid2;

    if (!ncu_snmp_multi_req)
        return;
    ncu_snmp_multi_req[*id].oidp = (oid *)malloc(slen);
    oid2 = (oid *)ncu_snmp_multi_req[*id].oidp;
    memmove(oid2, oid1, slen);
    if (*id >= NCUSYS_NUMSNMPREQ
            && ncu_rect_snmp_id) {
        oid2[len - 1] =
                ncu_rect_snmp_id[(*id - NCUSYS_NUMSNMPREQ) / NCURECT_NUMSNMPREQ];
    }
    ncu_snmp_multi_req[*id].oidlen = len;
    ncu_snmp_multi_req[*id].type = SNMPGET_STRING;
    *id += 1;
}

static void
ncu_snmp_poll(struct pwr *pwr)
{
    struct eem *e;
    struct snmp_device *sd;
    size_t i, j, num_req;

    if (!(e = pwr->internal)
            || !(sd = pwr->snmp_device)
            || !enl_is_ssaddr_valid(&sd->ip_target)) {
        eem_get_done(NULL, 0, pwr);
        return;
    }
    if (pwr_lost(pwr)) {
        SNMPDevice_free_snmpget(sd);
        snmpget_multi_stop(e->sm);
    }
    if (SNMPDevice_update_snmpget(sd)) {
        num_req = NCUSYS_NUMSNMPREQ + NCURECT_NUMSNMPREQ * e->pwr->in_count;
        if (!(ncu_snmp_multi_req =
                (struct _snmpget_multi_req *)malloc(
                        num_req * sizeof(struct _snmpget_multi_req))))
            return;
        i = 0;
        ncu_add_snmp_request(OID(identModel_oid), &i);
        ncu_add_snmp_request(OID(identCtrlFWVersion_oid), &i);
        ncu_add_snmp_request(OID(identName_oid), &i);
        ncu_add_snmp_request(OID(identSerialNum_oid), &i);
        for (j = 0; j < e->pwr->in_count; j++) {
            ncu_add_snmp_request(OID(rectProdNum_oid), &i);
            ncu_add_snmp_request(OID(rectHWVersion_oid), &i);
            ncu_add_snmp_request(OID(rectSWVersion_oid), &i);
            ncu_add_snmp_request(OID(rectSerialNum_oid), &i);
            ncu_add_snmp_request(OID(rectIdent_oid), &i);
        }
        if ((e->sm = SNMPDevice_multi_new(sd,
                        (struct snmpget_multi_req *)ncu_snmp_multi_req,
                        num_req, ncu_snmp_done, pwr))) {
            snmpget_multi_start(e->sm);
        }
    }
    e->snmp_status = NCUSNMP_RECTNUM;
}

static void
ncu_snmp_rect_poll(struct pwr *pwr, const oid *op, size_t len, uint8_t pdu_type)
{
    struct eem *e;
    struct snmp_device *sd;
    if (!(e = pwr->internal)
            || !(sd = pwr->snmp_device)
            || !enl_is_ssaddr_valid(&sd->ip_target)) {
        eem_get_done(NULL, 0, pwr);
        return;
    }
    if (!op) {
        op = rectNumInstalled_oid;
        len = MAXCOUNT(rectNumInstalled_oid);
        pdu_type = SNMP_MSG_GET;
    }
    if (pwr_lost(pwr)) {
        SNMPDevice_free_snmpget(sd);
    }
    if (SNMPDevice_update_snmpget(sd)) {
        SNMPDevice_get_any(sd, pdu_type, op,
                len, ncu_snmp_rect_idx, pwr);
    }

}

static void
ncu_snmp_timeout(int fd UNUSED, short what UNUSED, void *arg)
{
    struct pwr *pwr = arg;
    struct eem *e;

    if (!pwr
            || !(e = pwr->internal))
        return;
    switch (e->snmp_status) {
    case NCUSNMP_RECTNUM:
        ncu_snmp_rect_poll(pwr, NULL, 0, 0);
        break;
    case NCUSNMP_RECTKEY:
        ncu_snmp_rect_poll(pwr, OID(rectKey_oid), SNMP_MSG_GETNEXT);
        break;
    case NCUSNMP_RECTID:
        ncu_snmp_rect_poll(pwr, OID(rectKeyI_oid), SNMP_MSG_GETNEXT);
        break;
    case NCUSNMP_MULTIRQ:
        ncu_snmp_poll(pwr);
        break;
    default:
        eem_get_done(NULL, 0, pwr);
        e->snmp_status = NCUSNMP_RECTNUM;
    }
}

static void
eemdev_read_next(struct pwr *pwr, struct eemdev *ed)
{
    struct eem *e = pwr->internal;
    while (!list_is_last(&ed->list, &e->device)) {
	ed = list_entry(ed->list.next, struct eemdev, list);
	if (ed->eemid == EEM_EIB_DISTRIBUTION_UNIT
	        && !pwr_type_ncu(pwr)) {
	    continue;
	}
	if (ed->eemid != EEM_UNKNOWN
	    && eemdev_read_callback[ed->eemid]) {
	    eemdev_read(ed);
	    return;
	}
    }
    pwr_update(pwr);
    pwr_alarm_start(pwr);
    if (eem_bat_current_negative(pwr)) {
        pwr->alarm[PWR_BAT_DISCHARGE].active =
                pwr->alarm[PWR_BAT_DISCHARGE].mask;
    }
    pwr->alarm[PWR_BATTERY_DISCONNECT].active |=
            (pwr->alarm[PWR_BATTERY_DISCONNECT].mask & 0x01);
    e->rb_rq_loop = false;
    eem_read_alarms(pwr, 0);
}

#ifdef EEM_ALARMSIM
static struct eemdev *
eemdev_find_id(struct eem *e, eemid_t id)
{
    struct eemdev *ed;
    list_for_each_entry (ed, &e->device, list) {
	if (ed->eemid == id) {
	    return ed;
	}
    }
    return NULL;
}
#endif

static void
eem_rc(char *buf, size_t len, void *arg)
{
    struct pwr *pwr = arg;
    struct eem *e = pwr->internal;
    struct eemdev *ed;
    const struct eem_alarm *al;
    unsigned long ul;
    uint8_t start;
    uint8_t block;
    size_t n;
    char *pa;
    char *pi;
    char *ps;
    char *pc;
    perceivedSeverity_t severity;
    uint8_t cat;
    ulong alarm_index;
    eemid_t eemid;
    pwr_alarm_t type;
    pwr_in_alarm_t type_in;
    uint16_t bit;
    float t;
    bool rectgr_alarm_mains = false;
    if (!buf) {
	goto out;
    }
    eem_dump(e, buf, len);
    if (!(pa = memchr(buf, '#', len))
	|| (ul = strtoul(buf, NULL, 16)) > 0xFF) {
	goto clear;
    }
    start = block = ul;
    buf[len] = '\0';
    while ((n = strcspn(++pa, "#*"))) {
	if ((pi = strchr(pa, '!'))
	    && (ps = strchr(++pi, '!'))
	    && (pc = strchr(++ps, '!'))
	    && (ed = eemdev_find(e, pi))) {
	    pc++;
	    cat = strtoul(pc, NULL, 16);
	    if (cat < MAXCOUNT(eem_severity)) {
		severity = eem_severity[cat];
	    } else {
		severity = warning;
	    }
	    if (ps[0] != 'I') {
		goto next;
	    }
	    alarm_index = strtoul(ps + 1, NULL, 16) / 2;
	    eem_printf("%d %.5s %u %.1s (%s)\n", block, pi, alarm_index, pc,
		       perceivedSeverity_get(severity));
	    eemid = eem_getid(pi);
	    if (eemid >= MAXCOUNT(eem_ae)
		|| alarm_index > eem_ae[eemid].count) {
		goto next;
	    }
	    al = &eem_ae[eemid].ae[alarm_index];
	    type = al->u.type;
	    bit = al->bit;
	    switch (eemid) {
	    case EEM_RECTIFIER:
	    case EEM_SOLAR_CONVERTER:
		pwr_in_alarm(e->pwr, ed->pwrp, al->u.in_type, bit,
			     severity, true);
		break;
	    case EEM_RECTIFIER_GROUP:
	    case EEM_SOLAR_CONVERTER_GROUP:
	    case EEM_LVD_UNIT:
		pwr_out_alarm(e->pwr, ed->pwrp, al->u.out_type, bit,
			      severity, true);
		break;
	    case EEM_BATTERY_GROUP:
	        if (pwr_type_ncu(pwr)
	                && type == PWR_BAT_HIGH_TEMP
	                && bit == 0) {
	            break;
	        }
	        goto def;
	    case EEM_SM_IO_IB2:
	        bit = 0;
	        alarm_index++;  /* incomming alarms start from index zero */
	        pwr_ext_alarm(e->pwr, alarm_index, (uint8_t) bit, severity, true);
	        break;
	    case EEM_SYSTEM:
	        if (pwr_type_ncu(pwr)) {
	            if (type == PWR_UNDERVOLTAGE      // Undervoltage 2
	                    && bit == 1) {
	                type = PWR_BATTERY_DISCONNECT;
	                severity = indeterminate;
	                pwr_alarm_additional(pwr, type, bit, severity,
	                        "Low voltage initiated", true);
	                break;
	            }
	            if (type == PWR_DOOR
	                    || type == PWR_EXT) {
	                break;
	            }
	        } else {
	            goto def;
	        }
	    case EEM_DC_DISTRIBUTION_FUSE_UNIT:
	        if (pwr_type_ncu(pwr)) {
	            al = &ncu_remapping_DC_Distribution_Fuse_Unit_ae[alarm_index];
	            type = al->u.type;
	            bit = al->bit;
	            severity = indeterminate;
	            if (type == PWR_BATTERY_DISCONNECT) {
	                pwr_alarm_additional(pwr, type, bit, severity,
	                        "Contactor open", true);
	                break;
	            }
	            goto def;
	        }
	    case EEM_RECTIFIER_AC:
	        if (pwr_type_ncu(pwr)) {
	            type_in = al->u.in_type;
	            if (type_in == PWR_IN_MAINS) {
	                rectgr_alarm_mains = true;
	                break;
	            }
	        }
	    default:
def:        pwr_alarm(e->pwr, type, bit, severity, true);
	    break;
	    }
	}
    next:
	pa += n;
	block++;
    }
    //Rectifier group alarms obtained from the individual rectifiers
    if (e) {
	ed = eemdev_find(e, "0200");
	if (ed) {
	    eem_rectifiers_alarms(pwr, ed, rectgr_alarm_mains);
	}
    }
    if (block - start > 9) {
	eem_read_alarms(pwr, block);
    } else { /* Finished reading alarm list */
#ifdef EEM_ALARMSIM
	uint32_t r;
	size_t count;
	int i;
	for (i = 0; i < 10; i++) {
	    r = random();
	    severity = eem_severity[r % MAXCOUNT(eem_severity)];
	    r /= MAXCOUNT(eem_severity);
	    eemid = r % MAXCOUNT(eem_ae);
	    r /= MAXCOUNT(eem_ae);
	    if (!(count = eem_ae[eemid].count)) {
		continue;
	    }
	    alarm_index = r % count;
	    r /= count;
	    if ((ed = eemdev_find_id(e, eemid))) {
		al = &eem_ae[eemid].ae[alarm_index];
		type = al->u.type;
		bit = al->bit;
		switch (eemid) {
		case EEM_RECTIFIER:
		case EEM_SOLAR_CONVERTER:
		    pwr_in_alarm(e->pwr, ed->pwrp, al->u.in_type, bit,
				 severity, true);
		    break;
		case EEM_RECTIFIER_GROUP:
		case EEM_SOLAR_CONVERTER_GROUP:
		case EEM_LVD_UNIT:
		    pwr_out_alarm(e->pwr, ed->pwrp, al->u.out_type, bit,
				  severity, true);
		    break;
		default:
		    pwr_alarm(e->pwr, type, bit, severity, true);
		    break;
		}
	    }
	}
#endif
    clear:
	pwr_alarm_clear_inactive(pwr);
    out:
	t = pwr_time(pwr);
	pwr->run_time += t;
	eem_set_blvd_dm(pwr);
	eemdev_read_first(e);
    }
}

static void
eem_read_alarms(struct pwr *pwr, int block)
{
    char command[5];
    snprintf(command, sizeof command, "RC%02X", block);
    eemr_new(pwr, command, eem_rc, pwr);
}

static void
eem_scan(int fd UNUSED, short what UNUSED, void *arg)
{
    struct pwr *pwr = arg;
    eemr_new(pwr, "RI", eem_ri, pwr);
}

const char *
eem_open(struct pwr *pwr)
{
    MO_instance *ins, *child, *rect, *insr;
    struct eem *e;
    char buf[4];
    const ip_ssaddr_t *ap = &pwr->ip.addr_port;
    uint8_t cc_id = pwr->ip.sub_addr;

    if ((e = pwr->internal)) {
	if (enl_is_ssaddr_valid(ap)
	        && enl_is_ssaddr_eq(ap, &e->dest)) { /* No change */
	    goto out;
	}
	eem_close(e);
    } else {
	if (!(e = MALLOCP(struct eem))) {
	    return out_of_memory;
	}
	ZERO(e);
	INIT_LIST_HEAD(&e->device);
	INIT_LIST_HEAD(&e->physical);
	INIT_LIST_HEAD(&e->queue);
	pwr->internal = e;
	e->timeout_event = evtimer_new(event_base, eemr_timeout, pwr);
	e->scan_event = evtimer_new(event_base, eem_scan, pwr);
	e->event = evtimer_new(event_base, eem_connect_timeout, pwr);
	e->snmp_event = evtimer_new(event_base, ncu_snmp_timeout, pwr);
    }
    e->pwr = pwr;
    e->dest = *ap;
    if (e->state == EEM_INACTIVE && e->event) {
	evtimer_sec_add(e->event, 5);
    }
out:
    snprintf(buf, sizeof buf, "%02X", cc_id);
    memmove(e->cc_id, buf, sizeof e->cc_id);
    if (!(ins = pwr->ins)) {
	return NULL;
    }

    MO_for_children (ins, child) {
        if (!(insr = MO_followRef(child))) {
            continue;
        }
        switch (insr->class_index) {
        case Rectifiers_index:
            eemdev_new(e, child->instanceId, true);
            MO_for_children (child, rect) {
                if (rect->class_index == Rectifier_index) {
                    eemdev_new(e, rect->instanceId, false);
                }
            }
            break;
        case LoadDisconnect_index:
            if (!strncmp(child->instanceId, "Load1",
                    sizeof (child->instanceId))) {
                eemdev_new(e, "0701", false);
            } else if (!strncmp(child->instanceId, "Load2",
                    sizeof (child->instanceId))) {
                eemdev_new(e, "0702", false);
            }
            break;
        case Equipment_index:
        case SolarConverters_index:
        case SolarConverter_index:
            eemdev_new(e, child->instanceId, false);
            break;
        case EEMUnit_index:
            if (insr->parent &&
                    (insr->parent->class_index == EEM_index)) {
                EEM_set(insr->parent, e);
            }
            break;
        default:
            break;
        }
    }
    if ((ins = pwr_find_battery_profile_ins(pwr))
            && ins->class_index == BatteryProfile_index) {
        eemdev_new(e, EEM_LVD_UNIT_ID3, false);
    }
    return NULL;
}

struct eemdev *
eemdev_find_ins(MO_instance *ins)
{
    struct pwr *pwr;
    struct eemdev *ed;
    if ((pwr = pwr_parent(ins))
	&& (ed = eemdev_find(pwr->internal, ins->instanceId))) {
	return ed;
    }
    return NULL;
}

struct eemdata *
eemdata_find_ins(MO_instance *ins)
{
    struct eemdev *ed;
    if ((ed = eemdev_find_ins(ins))) {
	return &ed->data;
    }
    return NULL;
}

struct eemdata *
eemdata_find(struct pwr *pwr, const char *id)
{
    struct eem *e;
    struct eemdev *ed;
    if ((e = pwr->internal)
	&& (ed = eemdev_find(e, id))) {
	return &ed->data;
    }
    return NULL;
}

static void
eem_wb(char *buf, size_t len, void *arg)
{
    struct eemdev *ed = arg;
    eem_dump(ed->eem, buf, len);
    /* FIX */
}

static void
eemr_dump(char *buf, size_t len, void *arg)
{
    struct eem *e = arg;
    eem_dump(e, buf, len);
}

void
eem_command(struct pwr *pwr, const char *buf)
{
    struct eem *e;
    if ((e = pwr->internal)) {
	eemr_new(pwr, buf, eemr_dump, e);
    }
}

static uint32_t
eem_ftou(float f)
{
    int32_t m;
    uint32_t e;
    union {
	uint32_t u;
	float f;
    } u;
    uint32_t uh;
    if (!f) {
	return 0;
    }
    u.f = f;
    uh = u.u;
    m = (uh & 0x7FFFFE) | 0x800000;
    if (f < 0) {
	m = -m;
	m &= 0xFFFFFF;
	m |= 0x1000000;
    }
    m <<= 7;
    e = ((uh >> 23) - 126) & 0xFF;
    return m | e;
}

static void
eem_write(struct eemdev *ed, float *a, size_t na, uint8_t *d, size_t nd)
{
    char buf[1024];
    size_t maxlen;
    size_t len;
    size_t i;
    float f;
    char *p;
    uint8_t bit;
    uint8_t b;
    len = snprintf(buf, sizeof buf, "WB%.4s!", ed->id);
    p = buf + len;
    maxlen = sizeof buf - len;


    for (i = 0; i < na; i++) {
	f = a[i];
	if (isnan(f)) {
	    len = snprintf(p, maxlen, "7FFFFF80");
	} else {
	    len = snprintf(p, maxlen, "%08X", eem_ftou(a[i]));
	}
	if (len >= maxlen) {
	    return;
	}
	p += len;
	maxlen -= len;
    }
    *p++ = '!';
    maxlen--;
    bit = 0;
    b = 0;
    for (i = 0; i < nd; i++) {
	b <<= 1;
	if (d[i]) {
	    b |= 1;
	}
	if (++bit == 4) {
	    len = snprintf(p, maxlen, "%X", b);
	    if (len >= maxlen) {
		return;
	    }
	    p += len;
	    maxlen -= len;
	    bit = 0;
	    b = 0;
	}
    }
    if (bit) {
	b <<= 4 - bit;
	len = snprintf(p, maxlen, "%X", b);
	if (len >= maxlen) {
	    return;
	}
	p += len;
    }
    *p = '\0';
    eemr_new(ed->eem->pwr, buf, eem_wb, ed);
}

/**
 * @brief Sets the Analog Output parameter of the EEM battery group at the index specified by argument "ao_index"
 *
 * @param pwr       Instace of the PowerSystem MO
 * @param ao_index  Index of the EEM parameter in Analogue Output (ao) of the EEM battery group
 * @param value     Value of the EEM parameter
 */
static void
eem_set_battery_analog_output(struct pwr *pwr, enum eem_battey_ao_index ao_index, float value)
{
    const struct eem_device *device = &eem_blocks[EEM_BATTERY_GROUP];
    struct eemdev *ed;
    struct eem *e;
    size_t ao_count;
    float *f, *ao;
    size_t len;

    if (!(e = pwr->internal)
     || !(ed = eemdev_find(e, BATTERY_GROUP_ID))
     || !(ao = ed->data.ao_value)
     || !(ao_count = device->ao_count)
     || !((int)ao_index >= 0 && ao_index < ao_count) //NOTE: cast to int needed because powerpc-gcc implements enum as unsigned int (which causes warning in ">= 0" comparison)
     || (len = ao_count * sizeof f[0]) > 4096
     || !(f = alloca(len)))
    {
        return;
    }

    memmove(f, ao, len);
    f[ao_index] = value;
    eem_write(ed, f, ao_count, NULL, 0);
}

void
eem_set_equalize_charge_stop_delay_time(struct pwr *pwr, float value)
{
    const struct eem_device *device = &eem_blocks[EEM_BATTERY_GROUP];
    struct eemdev *ed;
    struct eem *e;
    size_t ao_count;
    float *f, *ao;
    size_t len;
    if (!(e = pwr->internal)
	|| !(ed = eemdev_find(e, BATTERY_GROUP_ID))
	|| !(ao = ed->data.ao_value)
	|| !(ao_count = device->ao_count)
	|| (len = ao_count * sizeof f[0]) > 4096
	|| !(f = alloca(len))) {
	return;
    }
    memmove(f, ao, len);
    f[44] = value;
    eem_write(ed, f, ao_count, NULL, 0);
}

void
eem_set_equalize_charge_stop_current(struct pwr *pwr, float value)
{
    const struct eem_device *device = &eem_blocks[EEM_BATTERY_GROUP];
    struct eem *e;
    struct eemdev *ed;
    size_t ao_count;
    float *f, *ao;
    size_t len;
    if (!(e = pwr->internal)
	|| !(ed = eemdev_find(e, BATTERY_GROUP_ID))
	|| !(ao = ed->data.ao_value)
	|| !(ao_count = device->ao_count)
	|| (len = ao_count * sizeof f[0]) > 4096
	|| !(f = alloca(len))) {
	return;
    }
    memmove(f, ao, len);
    f[45] = value;
    eem_write(ed, f, ao_count, NULL, 0);
}

void
eem_set_cyclic_equalize_charge_interval(struct pwr *pwr, float value)
{
    const struct eem_device *device = &eem_blocks[EEM_BATTERY_GROUP];
    struct eem *e;
    struct eemdev *ed;
    size_t ao_count;
    float *f, *ao;
    size_t len;
    if (!(e = pwr->internal)
	|| !(ed = eemdev_find(e, BATTERY_GROUP_ID))
	|| !(ao = ed->data.ao_value)
	|| !(ao_count = device->ao_count)
	|| (len = ao_count * sizeof f[0]) > 4096
	|| !(f = alloca(len))) {
	return;
    }
    memmove(f, ao, len);
    f[14] = value;
    eem_write(ed, f, ao_count, NULL, 0);
}

void
eem_set_boost_charge(struct pwr *pwr, bool boost)
{
    const struct eem_device *device = &eem_blocks[EEM_BATTERY_GROUP];
    struct eem *e;
    struct eemdev *ed;
    size_t do_count;
    uint8_t *d, *dout;
    size_t len;
    if (!(e = pwr->internal)
	|| !(ed = eemdev_find(e, BATTERY_GROUP_ID))
	|| !(dout = ed->data.do_value)
	|| !(do_count = device->do_count)
	|| (len = do_count * sizeof d[0]) > 4096
	|| !(d = alloca(len))) {
	return;
    }
    memmove(d, dout, len);
    d[4] = boost;
    d[6] = !boost;
    eem_write(ed, NULL, 0, d, do_count);
}

void
eem_set_boost_voltage(struct pwr *pwr, float value)
{
    const struct eem_device *device = &eem_blocks[EEM_BATTERY_GROUP];
    struct eem *e;
    struct eemdev *ed;
    size_t ao_count;
    float *f, *ao;
    size_t len;
    if (!(e = pwr->internal)
    || !(ed = eemdev_find(e, BATTERY_GROUP_ID))
    || !(ao = ed->data.ao_value)
    || !(ao_count = device->ao_count)
    || (len = ao_count * sizeof f[0]) > 4096
    || !(f = alloca(len))) {
    return;
    }
    memmove(f, ao, len);
    f[8] = value;
    eem_write(ed, f, ao_count, NULL, 0);
}

void
eem_set_charge_curr(struct pwr *pwr, float value)
{
    const struct eem_device *device = &eem_blocks[EEM_BATTERY_GROUP];
    struct eem *e;
    struct eemdev *ed;
    size_t ao_count;
    float *f, *ao;
    size_t len;
    if (!(e = pwr->internal)
    || !(ed = eemdev_find(e, BATTERY_GROUP_ID))
    || !(ao = ed->data.ao_value)
    || !(ao_count = device->ao_count)
    || (len = ao_count * sizeof f[0]) > 4096
    || !(f = alloca(len))) {
    return;
    }
    memmove(f, ao, len);
    f[7] = value;
    eem_write(ed, f, ao_count, NULL, 0);
}

void
eem_set_loadD(struct pwr *pwr, uint8_t idx)
{
    const struct eem_device *device = &eem_blocks[EEM_LVD_UNIT];

    struct eemdev *ed;
    struct eem *e;
    size_t ao_count, len;
    float *data, *ao;
    if (!(e = pwr->internal) || !IS_IN_RANGE(idx, MAIN_LD_CONID, PRIO_LD_CONID)
            || !(ed = eemdev_find(e, ssprintf("070%d", idx)))) {
        return;
    }
    if (!(ao_count = device->ao_count)
    || !(ao = ed->data.ao_value)
    || (len = ao_count * sizeof data[0]) > 4096
    || !(data = alloca(len))) {
    return;
    }
    memmove(data, ao, len);
    if (!isnan(pwr->config.f[dc_load_disconnect_time1 + idx - 1])) {
    data[0] = pwr->config.f[dc_load_disconnect_time1 + idx - 1];
    }
    if (!isnan(pwr->config.f[dc_load_disconnect_volt1 + idx - 1])) {
    data[1] = pwr->config.f[dc_load_disconnect_volt1 + idx - 1];
    }
    data[2] = LVD_RECONNECT_TIME;
    if (!isnan(pwr->config.f[dc_load_reconnect_volt1 + idx - 1])) {
    data[3] = pwr->config.f[dc_load_reconnect_volt1 + idx - 1];
    }
    eem_write(ed, data, ao_count, NULL, 0);
}

void
eem_set_blvd(struct pwr *pwr, pwr_config_float_t type, float f)
{
    const struct eem_device *device = &eem_blocks[EEM_LVD_UNIT];

    struct eemdev *ed;
    struct eem *e;
    size_t ao_count, len, id;
    float *data, *ao;

    if (!(e = pwr->internal) ||
            !(ed = eemdev_find(e, ssprintf("070%d", BLVD_LD_CONID)))) {
        return;
    }
    if (!(ao_count = device->ao_count)
    || !(ao = ed->data.ao_value)
    || (len = ao_count * sizeof data[0]) > 4096
    || !(data = alloca(len))) {
    return;
    }
    memmove(data, ao, len);
    switch (type) {
    case bat_disconnect_time:
        id = 0;
        break;
    case bat_disconnect_voltage:
        id = 1;
        break;
    case bat_reconnect_voltage:
        id = 3;
        break;
    default:
        return;
    }
    data[id] = f;
    data[2] = LVD_RECONNECT_TIME;
    eem_write(ed, data, ao_count, NULL, 0);
}

void
eem_set_blvd_dm(struct pwr *pwr)
{
    const struct eem_device *device = &eem_blocks[EEM_LVD_UNIT];
    struct eem *e;
    struct eemdev *ed;
    size_t len;
    size_t do_count;
    uint8_t *d, *dout;

    if (!(e = pwr->internal)
            || !(ed = eemdev_find(e, ssprintf("070%d", BLVD_LD_CONID)))
            || !(dout = ed->data.do_value)
            || !(do_count = device->do_count)
            || (len = do_count * sizeof d[0]) > 4096
            || !(d = alloca(len))) {
        return;
    }
    memmove(d, dout, len);
    // Always set BLVD disconnect method to Voltage
    if (d[2] != EEM_VOLTAGE_DM) {
        d[2] = EEM_VOLTAGE_DM;
    } else {
        return;
    }
    eem_write(ed, NULL, 0, d, do_count);
}

void
eem_set_loadD_dm(struct pwr *pwr, uint8_t idx)
{
    const struct eem_device *device = &eem_blocks[EEM_LVD_UNIT];
    struct eem *e;
    struct eemdev *ed;
    struct loadD *ld;
    size_t len;
    size_t do_count;
    uint8_t *d, *dout;

    if (!(e = pwr->internal) ||!(ld = IS_LLVD(idx - 1))
            || !(ed = eemdev_find(e, ssprintf("070%d", idx)))
            || !(dout = ed->data.do_value)
            || !(do_count = device->do_count)
            || (len = do_count * sizeof d[0]) > 4096
            || !(d = alloca(len))) {
        return;
    }
    memmove(d, dout, len);
    switch (ld->dm) {
    case BatteryVoltage:
        d[2] = EEM_VOLTAGE_DM;
        break;
    case Time:
        d[2] = EEM_TIME_DM;
        break;
    default:
        return;
    }
    eem_write(ed, NULL, 0, d, do_count);
}

void
eem_set_contactors(struct pwr *pwr, uint8_t set, uint8_t clr)
{
    const struct eem_device *device = &eem_blocks[EEM_SYSTEM];
    struct eem *e;
    struct eemdev *ed;
    size_t do_count;
    uint8_t *d;
    if (!(e = pwr->internal)
	|| !(ed = eemdev_find(e, CSU_ID))
	|| !(do_count = device->do_count)
	|| !(d = ed->data.do_value)) {
	return;
    }
    if (set & 1) {
	d[8] = 1;
    } else if (clr & 1) {
	d[8] = 0;
    }
    if (set & 2) {
	d[10] = 1;
    } else if (clr & 2) {
	d[10] = 0;
    }
    if (set & 4) {
	d[12] = 1;
    } else if (clr & 4) {
	d[12] = 0;
    }
    eem_write(ed, NULL, 0, d, do_count);
}

void
eem_set_float_voltage(struct pwr *pwr, float value)
{
    const struct eem_device *device = &eem_blocks[EEM_SYSTEM];
    struct eem *e;
    struct eemdev *ed;
    size_t ao_count;
    float *f, *ao;
    size_t len;

    ao_count = device->ao_count;
    if (!pwr_type_ncu(pwr)) {       // support for ACU+
        ao_count = 14;
    }
    if (!(e = pwr->internal)
    || !(ed = eemdev_find(e, CSU_ID))
    || !(ao = ed->data.ao_value)
    || !ao_count
    || (len = ao_count * sizeof f[0]) > 4096
    || !(f = alloca(len))) {
    return;
    }
    memmove(f, ao, len);
    f[0] = value;
    eem_write(ed, f, ao_count, NULL, 0);
}

void
eem_set_eco_mode(struct pwr *pwr, bool eco_mode)
{
    const struct eem_device *device = &eem_blocks[EEM_RECTIFIER_GROUP];
    struct eem *e;
    struct eemdev *ed;
    size_t len;
    size_t do_count;
    uint8_t *d, *dout;
    if (!(e = pwr->internal)
	|| !(ed = eemdev_find(e, RECTIFIER_GROUP_ID))
	|| !(dout = ed->data.do_value)
	|| !(do_count = device->do_count)
	|| (len = do_count * sizeof d[0]) > 4096
	|| !(d = alloca(len))) {
	return;
    }
    memmove(d, dout, len);
    d[6] = eco_mode;
    eem_write(ed, NULL, 0, d, do_count);
}

void
eem_rectifier_enable(struct pwr *pwr, struct pwr_in *in, bool enable)
{
    const struct eem_device *device = &eem_blocks[EEM_RECTIFIER];
    struct eem *e;
    struct eemdev *ed;
    size_t len;
    size_t do_count;
    uint8_t *d, *dout;
    if (!(e = pwr->internal)
	|| !(ed = eemdev_find(e, in->id))
	|| !(dout = ed->data.do_value)
	|| !(do_count = device->do_count)
	|| (len = do_count * sizeof d[0]) > 4096
	|| !(d = alloca(len))) {
	return;
    }
    memmove(d, dout, len);
    d[0] = !enable;
    eem_write(ed, NULL, 0, d, do_count);
}

bool
eem_get_bat(void *dev, float *up, float *ip, float *tp, float *cp, float *np)
{
    struct eemdev *ed = dev;
    struct eemdev *ed0;
    float *ai, *ao, f, a;
    if (!ed) {
	return false;
    }
    if ((ai = ed->data.ai_value)) {
	*ip = ai[1];
	*tp = ai[2];
	*cp = ai[3];
	f = ai[0];
    } else {
	*ip = NAN;
	*tp = NAN;
	*cp = NAN;
	f = NAN;
    }
    if (isnan(f) || (a = fabsf(f)) < 1 || a > 1000) {
	if ((ed0 = eemdev_find(ed->eem, CSU_ID))
	    && (ai = ed0->data.ai_value)) {
	    f = ai[0];		/* System Voltage */
	}
    }
    *up = f;
    if ((ed = eemdev_find(ed->eem, BATTERY_GROUP_ID))
	&& (ao = ed->data.ao_value)) {
	f = ao[18];		/* Nominal Battery Capacity */
    } else {
	f = 0;
    }
    *np = f;
    return true;
}

void
eem_set_bat_cap (struct pwr *pwr, float f)
{
    const struct eem_device *device = &eem_blocks[EEM_BATTERY_UNIT];
    struct eemdev *ed;
    struct eem *e;
    size_t ao_count, len;
    float *ao, *data;

    if (!(e = pwr->internal)
            || !(ed = eemdev_find(e, BATTERY_UNIT_ID))
            || !(ao_count = device->ao_count)
            || !(ao = ed->data.ao_value)
            || (len = ao_count * sizeof data[0]) > 4096
            || !(data = alloca(len))) {
        return;
    }
    memmove(data, ao, len);
    if (!isnan(f)) {
        data[0] = Ah(f);
    }
    eem_write(ed, data, ao_count, NULL, 0);
}

void
eem_set_system(struct pwr *pwr, pwr_config_float_t type, float f)
{
    const struct eem_device *device = &eem_blocks[EEM_SYSTEM];
    struct eemdev *ed;
    struct eem *e;
    size_t ao_count;
    float *ao;

    if (!(e = pwr->internal) || !(ed = eemdev_find(e, CSU_ID))) {
        return;
    }
    ao_count = device->ao_count;
    if (!pwr_type_ncu(pwr)) {       // support for ACU+
        ao_count = 14;
    }
    if (!ao_count
            || !(ao = ed->data.ao_value)) {
        return;
    }
    if (!isnan(f)) {
        switch (type) {
            case dc_volt_max:
                ao[5] = f;
                break;
            case bat_high_env_temp:
                ao[9] = f;
                if (pwr_type_ncu(pwr))
                    ao[20] = f;
                break;
            case bat_low_env_temp:
                ao[10] = f;
                if (pwr_type_ncu(pwr))
                    ao[21] = f;
                break;
            case batt1_high_temp:
                ao[14] = f;
                break;
            case batt1_low_temp:
                ao[15] = f;
                break;
            case batt2_high_temp:
                ao[16] = f;
                break;
            case batt2_low_temp:
                ao[17] = f;
                break;
            case batt3_high_temp:
                ao[18] = f;
                break;
            case batt3_low_temp:
                ao[19] = f;
                break;
            default:
                break;
        }
    }
    eem_write(ed, ao, ao_count, NULL, 0);
}

void
eem_set_lvd_group(struct pwr *pwr, pwr_config_float_t type, float f)
{
    const struct eem_device *device = &eem_blocks[EEM_LVD_GROUP];
    struct eemdev *ed;
    struct eem *e;
    size_t ao_count;
    float *ao;

    if (!(e = pwr->internal) || !(ed = eemdev_find(e, LVD_GROUP_ID))) {
        return;
    }
    if (!(ao_count = device->ao_count)
            || !(ao = ed->data.ao_value)) {
        return;
    }
    if (!isnan(f)) {
        switch (type) {
            case bat_high_temp_major:
                ao[8] = f;
                break;
            case bat_high_temp_minor:
                ao[9] = f;
                break;
            default:
                break;
        }
    }
    eem_write(ed, ao, ao_count, NULL, 0);
}

#define CURL_PATH "/usr/bin/curl"
#define EEM_ARG(v, n) v[MAXCOUNT(eem_curl_argv) + n]
#define EEM_COOKIE "/tmp/eem.cookie"
#define EEM_SESSION "session_ID"

static char *const eem_curl_argv[] = {
    "curl",
    "--user-agent", "Mozilla/5.0 (compatible; MSIE 9.0)",
    "--cookie-jar", EEM_COOKIE,
};

static char **
eem_build_argv(int count)
{
    char **argv;
    if (!(argv = calloc(MAXCOUNT(eem_curl_argv) + count + 1, sizeof argv[0]))) {
	return NULL;
    }
    memmove(argv, eem_curl_argv, sizeof eem_curl_argv);
    return argv;
}

static void
eem_restart_done(int wait_status, void *arg UNUSED)
{
    const char *status = NULL;
    char buf[32];
    if (wait_status) {
	status = ssprintf("curl: %s",
			  wait_status_string(buf, sizeof buf, wait_status));
    }
    command_complete(status);
}

//#define CURL_OUTPUT "/tmp/curl%d"

static void
eem_curl_exec(char **argv)
{
    int fd;
#ifdef CURL_OUTPUT
    char file[32];
#endif
    if ((fd = open("/dev/null", O_WRONLY)) >= 0) {
	dup2(fd, 1);
	dup2(fd, 2);
	close(fd);
    }
#ifdef CURL_OUTPUT
    snprintf(file, sizeof file, CURL_OUTPUT, getpid());
    if ((fd = open(file, O_WRONLY|O_CREAT, 0640))) {
	dup2(fd, 1);
	close(fd);
    }
#endif
    execv(CURL_PATH, argv);
    debug_printf("Can't exec %s\n", strperror(CURL_PATH));
    exit(1);
}

static void
eem_login_done(int wait_status, void *arg)
{
    struct eem *e = arg;
    char *session = NULL;
    char buf[256];
    char *p;
    FILE *f;
    char **argv;
    char url[256];
    char acuIP[IP_MAX_ADDRSTRLEN + 8];
    int pid;
    if (wait_status
	|| !(f = fopen(EEM_COOKIE, "r"))) {
	return;
    }
    while (fgets(buf, sizeof buf, f)) {
	if ((p = strstr(buf, EEM_SESSION))) {
	    p += strlen(EEM_SESSION);
	    p += strspn(p, " \t");
	    chomp(p);
	    session = strdupa(p);
	    break;
	}
    }
    fclose(f);
    if (!(argv = eem_build_argv(7))) {
	command_complete(out_of_memory);
	return;
    }
    enl_addr2uristr(&e->dest, buf);
    snprintf(acuIP, sizeof acuIP, "acuIP=%s", buf);
    snprintf(url, sizeof url, "http://%s/cgi-bin/web_cgi_control.cgi", buf);
    snprintf(buf, sizeof buf,
	     "equip_ID=-2&signal_type=-2&signal_id=-2&control_value=-2&"
	     "control_type=11&sessionId=%s&language_type=0", session);
    EEM_ARG(argv, 0) = "--cookie";
    EEM_ARG(argv, 1) = EEM_COOKIE;
    EEM_ARG(argv, 2) = "--cookie";
    EEM_ARG(argv, 3) = acuIP;
    EEM_ARG(argv, 4) = "--data";
    EEM_ARG(argv, 5) = buf;
    EEM_ARG(argv, 6) = url;
    if (!(pid = child_new(eem_restart_done, e))) {
	eem_curl_exec(argv);
    }
    free(argv);
    if (pid < 0) {
	command_complete(ssprintf("%s: %m", __func__));
    }
}

static void
eem_base64_encode(const char *src, char *dst, size_t dstlen)
{
    size_t len = strlen(src);
    size_t pad;
    char *s;
    if ((pad = len % 3)) {
	len += 3 - pad;		/* Pad with NUL:s */
    }
    s = alloca(len + 1);
    memset(s, 0, len);
    strmcpy(s, src, len + 1);
    base64_encode(s, len, dst, dstlen);
}

const char *
eem_restart(struct pwr *pwr)
{
    struct eem *e = pwr->internal;
    char **argv;
    char url[128];
    char data[256];
    char username[32];
    char password[32];
    int pid;
    if (!(argv = eem_build_argv(3))) {
	return out_of_memory;
    }
    enl_addr2uristr(&e->dest, data);
    snprintf(url, sizeof url, "http://%s/cgi-bin/web_cgi_main.cgi", data);
    eem_base64_encode(pwr->username ?: "admin", username, sizeof username);
    eem_base64_encode(pwr->password ?: "1", password, sizeof password);
    snprintf(data, sizeof data, "user_name=%s&user_password=%s&language_type=0",
	     username, password);
    debug_printf("%s\n", data);
    EEM_ARG(argv, 0) = "--data";
    EEM_ARG(argv, 1) = data;
    EEM_ARG(argv, 2) = url;
    if (!(pid = child_new(eem_login_done, e))) {
	eem_curl_exec(argv);
    }
    free(argv);
    if (pid < 0) {
	return ssprintf("%s: %m", __func__);
    }
    command_set_timeout(30);
    return NULL;
}

const char *
eem_set_config_float(struct pwr *pwr, pwr_config_float_t type, float f)
{
    switch (type) {
    case bat_float_charge_voltage:
        eem_set_float_voltage(pwr, f);
        break;
    case bat_boost_charge_voltage:
        eem_set_boost_voltage(pwr, f);
        break;
    case dc_load_disconnect_volt1:
    case dc_load_disconnect_time1:
    case dc_load_reconnect_volt1:
        eem_set_loadD(pwr, MAIN_LD_CONID);
        break;
    case dc_load_disconnect_volt2:
    case dc_load_disconnect_time2:
    case dc_load_reconnect_volt2:
        eem_set_loadD(pwr, PRIO_LD_CONID);
        break;
    case bat_disconnect_voltage:
        eem_check_undervolt(pwr, BLVD_LD_CONID, f);
        /* FALLTHRU */
    case bat_disconnect_time:
    case bat_reconnect_voltage:
        eem_set_blvd(pwr, type, f);
        break;
    case bat_nominal_capacity:
        eem_set_bat_cap(pwr, f);
        break;
    case dc_volt_max:
    case bat_high_env_temp:
    case bat_low_env_temp:
    case batt1_high_temp:
    case batt2_high_temp:
    case batt3_high_temp:
    case batt1_low_temp:
    case batt2_low_temp:
    case batt3_low_temp:
        eem_set_system(pwr, type, f);
        break;
    case bat_high_temp_major:
    case bat_high_temp_minor:
        eem_set_lvd_group(pwr, type, f);
        break;
    case bat_charge_curr_max:
        eem_set_charge_curr(pwr, f);
        break;
    case bat_test_term_time:
        eem_set_battery_analog_output(pwr, IDX_EEM_BATTERY_TEST_TIME, f);
        break;
    case bat_test_term_capacity:
        eem_set_battery_analog_output(pwr, IDX_EEM_USED_CAPACITY_LIMIT, f);
        break;
    case bat_low_voltage_minor:
        eem_set_undervolt(pwr, MAIN_LD_CONID, f);
        break;
    case bat_low_voltage_major:
        eem_set_undervolt(pwr, BLVD_LD_CONID, f);
        break;
    default:
        break;
    }
    return NULL;
}

bool
eem_can_set_config_float(pwr_config_float_t type)
{
    bool result;
    switch (type) {
    case bat_float_charge_voltage:
    case bat_boost_charge_voltage:
    case dc_load_disconnect_volt1:
    case dc_load_disconnect_time1:
    case dc_load_reconnect_volt1:
    case dc_load_disconnect_volt2:
    case dc_load_disconnect_time2:
    case dc_load_reconnect_volt2:
    case bat_disconnect_voltage:
    case bat_disconnect_time:
    case bat_reconnect_voltage:
    case bat_nominal_capacity:
    case dc_volt_max:
    case bat_high_env_temp:
    case bat_low_env_temp:
    case bat_high_temp_major:
    case bat_high_temp_minor:
    case bat_charge_curr_max:
    case batt1_high_temp:
    case batt2_high_temp:
    case batt3_high_temp:
    case batt1_low_temp:
    case batt2_low_temp:
    case batt3_low_temp:
    case bat_test_term_time:
    case bat_test_term_capacity:
    case bat_low_voltage_minor:
    case bat_low_voltage_major:
        result = true;
        break;
    default:
        result = false;
        break;
    }
    return result;
}

float
eem_round_config_float(struct pwr *pwr UNUSED, pwr_config_float_t type,
             float f)
{
    switch (type) {
    case bat_float_charge_voltage:
    case bat_boost_charge_voltage:
    case dc_load_disconnect_volt1:
    case dc_load_disconnect_time1:
    case dc_load_reconnect_volt1:
    case dc_load_disconnect_volt2:
    case dc_load_disconnect_time2:
    case dc_load_reconnect_volt2:
    case bat_disconnect_voltage:
    case bat_disconnect_time:
    case bat_reconnect_voltage:
    case bat_charge_curr_max:
    case bat_nominal_capacity:
    case dc_volt_max:
    case bat_high_env_temp:
    case bat_low_env_temp:
    case bat_high_temp_major:
    case bat_high_temp_minor:
    case batt1_high_temp:
    case batt2_high_temp:
    case batt3_high_temp:
    case batt1_low_temp:
    case batt2_low_temp:
    case batt3_low_temp:
    case bat_low_voltage_minor:
    case bat_low_voltage_major:
    f = round_decimals(f, 1000);
    break;
    default:
    break;
    }
    return f;
}

void
eem_debug(struct pwr *pwr, bool debug)
{
    struct eem *e;
    if ((e = pwr->internal)) {
	e->debug = debug;
    }
}

void
eem_delete(struct pwr *pwr)
{
    struct eem *e;
    struct snmp_device *sd;

    if ((e = pwr->internal)) {
        eem_free(e);
    }
    if ((sd = pwr->snmp_device)) {
        SNMPDevice_free_snmpget(sd);
    }
}

void
eem_loadD_volt_config_update(struct pwr *pwr, struct loadD *ld)
{
    struct eem *e;

    if (!(e = (struct eem *) pwr->internal) || !ld) {
        return;
    }

    switch (ld->contactorId) {
    case MAIN_LD_CONID:
        ld->disconnect_level =
                pwr->config.f[dc_load_disconnect_volt1];
        ld->disconnect_level_R =
                pwr->device_config.f[dc_load_disconnect_volt1];
        ld->reconnect_level =
                        pwr->config.f[dc_load_reconnect_volt1];
        ld->reconnect_level_R =
                pwr->device_config.f[dc_load_reconnect_volt1];
        break;
    case PRIO_LD_CONID:
        ld->disconnect_level =
                        pwr->config.f[dc_load_disconnect_volt2];
        ld->disconnect_level_R =
                pwr->device_config.f[dc_load_disconnect_volt2];
        ld->reconnect_level =
                        pwr->config.f[dc_load_reconnect_volt2];
        ld->reconnect_level_R =
                pwr->device_config.f[dc_load_reconnect_volt2];
        break;
    default:
        break;
    }
    ld->delay_value = pwr->config.f[dc_load_disconnect_time1 +
                                    ld->contactorId - 1];
    ld->delay_value_R = pwr->device_config.f[dc_load_disconnect_time1 +
                                    ld->contactorId - 1];
    if (pwr_type_ncu(pwr)) {
        eem_check_undervolt(pwr, ld->contactorId, ld->disconnect_level);
    }
}

void
eem_loadD_time_config_update(struct pwr *pwr, struct loadD *ld)
{
    struct eem *e;
    if (!(e = (struct eem *) pwr->internal)) {
        return;
    }
    switch (ld->contactorId) {
    case MAIN_LD_CONID:
        ld->delay_value = pwr->config.f[dc_load_disconnect_time1];
        ld->delay_value_R = pwr->device_config.f[dc_load_disconnect_time1];
        break;
    case PRIO_LD_CONID:
        ld->delay_value = pwr->config.f[dc_load_disconnect_time2];
        ld->delay_value_R = pwr->device_config.f[dc_load_disconnect_time2];
        break;
    default:
        break;
    }
    eem_loadD_volt_config_update(pwr, ld);
}

bool
eem_loadD_check_support(disconnectMethod_t dm, counter_state_t cs UNUSED)
{
    switch (dm) {
    case BatterySoC:
    case RefTrigger:
        return true;
    default:
        break;
    }
    return false;
}

/*
 * Setting Undervoltage level (system AO parameter)
 */
void
eem_set_undervolt(struct pwr *pwr, port_t contactorId, float value)
{
    struct eem *e;
    const struct eem_device *device = &eem_blocks[EEM_SYSTEM];
    struct eemdev *ed = NULL;
    size_t ao_count, len;
    float *ao, *f = NULL;

    ao_count = device->ao_count;
    if (!pwr_type_ncu(pwr)) {       // support for ACU+
        ao_count = 14;
    }
    if (!pwr
            || !(e = pwr->internal)
            || !(ed = eemdev_find(e, CSU_ID))
            || !(ao = ed->data.ao_value)
            || !ao_count
            || (len = ao_count * sizeof f[0]) > 4096
            || !(f = alloca(len))
            || isnan(value)) {
        return;
    }
    memmove(f, ao, len);
    switch(contactorId) {
    case MAIN_LD_CONID:
        f[EEMIDX_UNDERVOLT1] = value;
        break;
    case BLVD_LD_CONID:
        f[EEMIDX_UNDERVOLT2] = value;
        break;
    default:
        return;
    }
    eem_write(ed, f, ao_count, NULL, 0);
}

void
eem_check_undervolt(struct pwr *pwr, port_t id, float f)
{
    float volt, lvd;
    pwr_config_float_t type;

    if (!pwr || isnan(f))
    {
        return;
    }
    switch (id)
    {
    case MAIN_LD_CONID:
        type = bat_low_voltage_minor;
        lvd = f + UNDERVOLT1_DIFF;
        break;
    case BLVD_LD_CONID:
        type = bat_low_voltage_major;
        lvd = f;
        break;
    default:
        return;
    }
    if (!isnan(volt = pwr->device_config.f[type])
            && (round_decimals(lvd, 1000)
                    != round_decimals(volt, 1000)))
    {
        pwr_set_float_local(pwr, type, lvd, pwr->ins, "underVoltage", false);
    }
}

/*
 * Helper function for setting loadDisconnect values
 */
const char *
eem_loadD_local_set(struct pwr *pwr, struct loadD *ld)
{
    switch (ld->dm) {
    case BatteryVoltage:
        if (!isnanf(ld->delay_value)
                || !isnanf(ld->delay_value_R)
                || (ld->delay_count != CounterUNKNOWN)) {
            ld->delay_value = NAN;
            ld->delay_value_R = NAN;
            ld->delay_value_prev = NAN;
            ld->delay_count = CounterUNKNOWN;
            config_notify();
            config_delayed_write(100);
        }
        switch (ld->contactorId) {
        case MAIN_LD_CONID:
            pwr_set_float_local(pwr, dc_load_disconnect_volt1,
                    ld->disconnect_level, ld->ins, "disconnectLevel",
                    false);
            pwr_set_float_local(pwr, dc_load_reconnect_volt1,
                    ld->reconnect_level, ld->ins, "reconnectLevel",
                    false);
            break;
        case PRIO_LD_CONID:
            pwr_set_float_local(pwr, dc_load_disconnect_volt2,
                    ld->disconnect_level, ld->ins, "disconnectLevel",
                    false);
            pwr_set_float_local(pwr, dc_load_reconnect_volt2,
                    ld->reconnect_level, ld->ins, "reconnectLevel",
                    false);
            break;
        default:
            return NULL;
        }
        eem_set_loadD_dm(pwr, ld->contactorId);
        break;
    case Time:
        if ((ld->delay_count != CounterUNKNOWN)) {
            ld->delay_count = CounterUNKNOWN;
        }
        switch (ld->contactorId) {
        case MAIN_LD_CONID:
            pwr_set_float_local(pwr, dc_load_disconnect_time1,
                    ld->delay_value, ld->ins, "delayValue", false);
            break;
        case PRIO_LD_CONID:
            pwr_set_float_local(pwr, dc_load_disconnect_time2,
                    ld->delay_value, ld->ins, "delayValue", false);
            break;
        default:
            return NULL;
        }
        eem_set_loadD_dm (pwr, ld->contactorId);
        break;
    default:
        break;
    }
    return NULL;
}

const char *
eem_handler_load_disc(struct pwr *pwr, struct loadD *ld, MO_instance *ins UNUSED)
{
    switch (ld->dm) {
    case BatteryVoltage:
        eem_loadD_volt_config_update(pwr,ld);
        break;
    case Time:
        eem_loadD_time_config_update(pwr, ld);
        break;
    default:
        break;
    }
    return NULL;
}

static const char *
eem_check_disconnect_range(float disconnectLevel, float reconnectLevel)
{
    if (DISC_VOLT_DEP(disconnectLevel)) {
        return "DisconnectLevel value out of range";
    }

    if (REC_VOLT_DEP(reconnectLevel)) {
        return "ReconnectLevel value out of range";
    }
    if (!isnanf(disconnectLevel) && !isnanf(reconnectLevel)
            && (disconnectLevel > reconnectLevel)) {
        return "LVD disconnect level can't be set higher than "
                "the reconnect level";
    }
    return NULL;
}

const char *
eem_check_BLVD(struct pwr *pwr, MO_instance *ins)
{
    struct eem *e;
    struct LoadDisconnect *lvd1 = NULL;
    struct LoadDisconnect *lvd2 = NULL;
    struct BatteryProfile *blvd = NULL;

    if(!pwr || !(pwr->enabled) || !(e = pwr->internal)) {
        return NULL;
    }
    if(!(blvd = MO_data(ins))) {
        return NULL;
    }
    //Check BLVD/LVD1 voltage
    if ((e->ld[0]) && (lvd1 = MO_data(e->ld[0]))) {
        if (lvd1->disconnectMethod == BatteryVoltage) {
            if (lvd1->disconnectLevel < blvd->lowVoltageDisconnect) {
                return ssprintf(
                        "%s voltage cannot be set below BLVD voltage",
                        e->ld[0]->instanceId);
            }
            if (lvd1->reconnectLevel < blvd->lowVoltageReconnect) {
                return ssprintf("%s reconnect voltage cannot be set below "
                        "BLVD reconnect voltage", e->ld[0]->instanceId);
            }
        }
    }
    //Check BLVD/LVD2 voltage
    if ((e->ld[1]) && (lvd2 = MO_data(e->ld[1]))) {
        if (lvd2->disconnectMethod == BatteryVoltage) {
            if (lvd2->disconnectLevel < blvd->lowVoltageDisconnect) {
                return ssprintf("%s voltage cannot be set below BLVD voltage",
                        e->ld[1]->instanceId);
            }
            if (lvd2->reconnectLevel < blvd->lowVoltageReconnect) {
                return ssprintf("%s reconnect voltage cannot be set below "
                        "BLVD reconnect voltage", e->ld[1]->instanceId);
            }
        }
    }
    return NULL;
}

const char *
eem_check_LVD(struct pwr *pwr, MO_instance *ins)
{
    struct eem *e;
    const char *err = NULL;
    struct LoadDisconnect *ldin = NULL;
    struct LoadDisconnect *lvd1 = NULL;
    struct LoadDisconnect *lvd2 = NULL;

    if(!pwr || !(pwr->enabled) || !(e = pwr->internal)) {
        return NULL;
    }
    //Check range values per LVD
    if (!(ldin = MO_data(ins))) {
        return NULL;
    }
    if ((err = eem_check_disconnect_range(ldin->disconnectLevel,
            ldin->reconnectLevel))) {
        return err;
    }
    if (DISC_TIME_DEP(ldin->delayValue)) {
        return "DelayValue value out of range";
    }

    if (!(e->ld[0]) || !(e->ld[1])
            || !(lvd1 = MO_data(e->ld[0]))
            || !(lvd2 = MO_data(e->ld[1]))) {
        return NULL;
    }

    if (lvd2->disconnectMethod == Time || lvd1->disconnectMethod == Time) {
        if(!isnanf(lvd1->delayValue) && !isnanf(lvd2->delayValue)) {
            if (lvd1->delayValue <= lvd2->delayValue) {
                return ssprintf("%s time cannot be set below %s time",
                        e->ld[0]->instanceId, e->ld[1]->instanceId);
            }
        }
    }
    if (lvd2->disconnectMethod == BatteryVoltage
            || lvd1->disconnectMethod == BatteryVoltage) {
        if (!isnanf(lvd1->disconnectLevel) && !isnanf(lvd2->disconnectLevel)
                && (lvd1->disconnectLevel < lvd2->disconnectLevel)) {
            return ssprintf("%s voltage cannot be set below %s voltage",
                    e->ld[0]->instanceId, e->ld[1]->instanceId);
        }
        if (lvd1 && !isnanf(lvd1->reconnectLevel) && !isnanf(lvd2->reconnectLevel)
                && (lvd1->reconnectLevel < lvd2->reconnectLevel)) {
            return ssprintf("%s reconnect voltage cannot be set below %s "
                    "reconnect voltage",
                    e->ld[0]->instanceId, e->ld[1]->instanceId);
        }
    }
    return NULL;
}

static void
eem_reuse_threshold(MO_instance *thr, MO_instance *parent)
{
    struct Threshold *thrp;
    if (!(thrp = MO_data(thr))) {
        assert(0);
    }
    if (!parent) {
        return;
    }
    if (thrp->thresholdId == 0) {
        thrp->thresholdId = thrh_get_unique_id(parent);
        MO_writeInstance(thr);
    }
}

static void
eem_create_system_thresholds(struct pwr *pwr)
{
    MO_instance *thr;
    if (pwr->init && pwr->sync) {
        if (!(thr = MO_findChild((pwr->ins), Threshold_index, "HighSystemVoltage"))) {
            if ((thr = pwr_thr_new(pwr, pwr->ins, "HighSystemVoltage", THRH_TYPE_HIGH,
                    "systemVoltage", "systemVoltage"))) {
                Threshold_no_alarm(thr);
            }
        } else {
            eem_reuse_threshold(thr, pwr->ins);
        }
        /* Battery disconnect thresholds */
        if ((pwr->inbat) &&
                (!(thr = MO_findChild((pwr->inbat), Threshold_index, "TempDisconnectHigh")))) {
            pwr_bat_temp_thr_new(pwr, "TempDisconnectHigh", THRH_TYPE_HIGH,
                    "Battery Disconnect High Temperature");
        } else {
            eem_reuse_threshold(thr, pwr->inbat);
        }
        if ((pwr->inbat) &&
                (!MO_findChild((pwr->inbat), Threshold_index, "TempReconnectHigh"))) {
            pwr_bat_temp_thr_new(pwr, "TempReconnectHigh", THRH_TYPE_HIGH,
                    "Battery Reconnect High Temperature");
        } else {
            eem_reuse_threshold(thr, pwr->inbat);
        }
    }
}
static void
eem_create_temp_threshold(struct pwr *pwr, temp_type_t type, size_t idx)
{
    MO_instance *tempsi, *thr;

    if (!pwr->init || !pwr->sync) return;

    /* Battery temperature thresholds */
    if (pwr_type_ncu(pwr) && pwr->inbat) {
        if ((tempsi = temps_getMO(pwr, idx, type))) {
            if (!(thr = MO_findNextChildOfClass(tempsi, Threshold_index, NULL))) {
                temps_thrh_new(pwr, tempsi, type);
            } else {
                eem_reuse_threshold(thr, tempsi);
            }
        }
    }
}

static void
eem_temp_sensor(struct pwr *pwr, float t, temp_type_t type, size_t idx)
{
    if (!pwr) return;
    if (t == EEM_TEMP_NOT_CONF) return;
    if (idx > NCU_NUM_BATT_TEMP_SENS) return;
    temp_read(pwr, idx, type, t);
    /* IF VertivNCU only */
    eem_create_temp_threshold(pwr, type, idx);
}

static void
ld_get_dev_conf(struct pwr *pwr,
                struct loadD *ld,
                pwr_config_float_t disc_volt,
                pwr_config_float_t recon_volt,
                pwr_config_float_t disc_time)
{
    switch(ld->dm) {
    case Time:
        ld->delay_value = ld->delay_value_R = pwr->device_config.f[disc_time];
        /* FALLTHRU */
    case BatteryVoltage:
        ld->disconnect_level = ld->disconnect_level_R = pwr->device_config.f[disc_volt];
        ld->reconnect_level = ld->reconnect_level_R = pwr->device_config.f[recon_volt];
        break;
    default:
        break;
    }
}

void
eem_loadD_get_device_config(struct pwr *pwr, struct loadD *ld)
{
    switch(ld->contactorId) {
    case 1:
        ld_get_dev_conf(pwr, ld, dc_load_disconnect_volt1,
                dc_load_reconnect_volt1, dc_load_disconnect_time1);
        break;
    case 2:
        ld_get_dev_conf(pwr, ld, dc_load_disconnect_volt2,
                dc_load_reconnect_volt2, dc_load_disconnect_time2);
        break;
    default:
        break;
    }
}

#ifdef MEMDEBUG
void
eem_cleanup(void)
{
}
#endif

void
eem_bat_test(struct pwr *pwr, bool active)
{
    const struct eem_device *device = &eem_blocks[EEM_BATTERY_GROUP];
    struct eem *e;
    struct eemdev *ed;
    size_t len;
    size_t do_count;
    uint8_t *d, *dout;

    if (!(e = pwr->internal)
    || !(ed = eemdev_find(e, BATTERY_GROUP_ID))
    || !(dout = ed->data.do_value)
    || !(do_count = device->do_count)
    || (len = do_count * sizeof d[0]) > 4096
    || !(d = alloca(len))) {
        return;
    }
    memmove(d, dout, len);
    d[BATT_TEST_START] = active;
    d[BATT_TEST_STOP] = !active;
    if (!!d[BATT_TEST_AUTO])
        d[BATT_TEST_AUTO] = false;
    eem_write(ed, NULL, 0, d, do_count);
    if (active)
        pwr_mode_set(pwr, PWR_MODE_TEST);
}

void
eem_thrh_value(struct pwr *pwr, struct thrh *thrh)
{
    if (!pwr || !thrh) {
        return;
    }
    if (!pwr->enabled) {
        return;
    }
    switch(thrh->portId) {
        case PORTID_BATT_TEMP_DISCONNECT_HIGH:
        case PORTID_BATT_TEMP_RECONNECT_HIGH:
            thrh->value = pwr->bat_temperature;
            break;
        default:
            thrh->value = NAN;
            break;
    }
}

static void
eem_lvd_block_autoconf(struct pwr *pwr, const char *id)
{
    const char *prob = NULL;
    uint32_t cont_id = 0;
    MO_instance *ldi = NULL;
    struct LoadDisconnect *ldp;
    struct eem *e = pwr->internal;

    if (isctype(id[3], DIGIT)) {
        cont_id = id[3] - '0';
    }

    if (IS_IN_RANGE(cont_id, 1, 2) && !MO_getref(&PLD(cont_id - 1))) {

        switch (cont_id) {
        case MAIN_LD_CONID:
            prob = (pwr_type_ncu(pwr)) ? HPM_DISCONNECT_1 : MAIN_DISCONNECT;
            break;
        case PRIO_LD_CONID:
            prob = (pwr_type_ncu(pwr)) ? HPM_DISCONNECT_2 : PRIO_DISCONNECT;
            break;
        case BLVD_LD_CONID:
            return;
        default:
            debug_printf("Invalid contactor ID: %u, bye bye..", cont_id);
            assert(0);
        }

        if (!(ldi = pwr_loadD_new(pwr, eem_lvd_name(pwr, cont_id)))) {
            debug_printf("Unsuccessful load disconnect instance update");
        }
        if ((ldp = MO_data(ldi))) {
            ldp->contactorId = cont_id;
            ldp->disconnectMethod = BatteryVoltage;
            if (!MO_writeInstance(ldi)) {
                config_delayed_write(60);
            }
        }
        MO_setref(&PLD(cont_id - 1), ldi);
        if (prob) {
        loadD_set_specific_problem(ldi, prob);
        }
    }
}

char *
eem_get_sys_serial(struct pwr *pwr)
{
    struct eem *e;
    if(!pwr || !(e = pwr->internal))
        return NULL;
    return e->SerialNum;
}

static const char *
eem_alarm_align(MO_instance *ins, void *arg)
{
    struct pwr *pwr = arg;
    size_t i,j;
    struct AlarmEntry *ae_data;
    MO_instance *parent, *child, *insr;
    struct alarm_defSpecificProblem *alarm_defSpecificProblem_list;
    if ((!ins)||(!pwr)) {
        return NULL;
    }
    for (i = 0; i < MAXCOUNT(pwr_alarm_defSpecificProblem_list); i++) {
        if (pwr->type == pwr_alarm_defSpecificProblem_list[i].type) {
            alarm_defSpecificProblem_list = pwr_alarm_defSpecificProblem_list[i].list;
            MO_for_children (ins, child) {
               if (!(insr = MO_followRef(child))) {
                        continue;
               }
               switch (insr->class_index) {
                  case AlarmEntry_index:
                     parent = insr->parent;
                     if (parent) {
                        for (j = 0; j < pwr_alarm_defSpecificProblem_list[i].size; j++) {
                            if ((parent->class_index == alarm_defSpecificProblem_list[j].class_index)
                               && (!strncmp(insr->instanceId, alarm_defSpecificProblem_list[j].instanceId, INSTANCE_ID_LENGTH))) {
                                   ae_data = MO_data(insr);
                                   if (ae_data) {
                                       strmcpy(ae_data->specificProblem,
                                       alarm_defSpecificProblem_list[j].specificProblem, sizeof(ae_data->specificProblem));
                                   }
                             }
                         }
                     }
                   break;
                   default:
                   break;
               }
            }
        }
    }
  return NULL;
}

static const char *
eem_threshold_alarmentry_update(MO_instance *ins, void *arg)
{
    struct pwr *pwr = arg;
    size_t i,j;
    MO_instance *child, *insr, *ins_del;
    bool delete_ins = true;
    if ((!ins)||(!pwr)) {
            return NULL;
    }
    switch (ins->class_index) {
       case Threshold_index:
           for (i = 0; i < MAXCOUNT(ncu_thr_allowed_alentry_list); i++) {
               if ((!(strncmp(ncu_thr_allowed_alentry_list[i].instanceId,
                       ins->instanceId,INSTANCE_ID_LENGTH))) && (ins->parent) &&
                       (ins->parent->class_index == ncu_thr_allowed_alentry_list[i].par_class_index)){
                   ins_del = NULL;
                   MO_for_children (ins, child) {
                         if (ins_del) {
                             MO_unlink(ins_del);
                             ins_del = NULL;
                         }
                         if (!(insr = MO_followRef(child))) {
                               continue;
                         }
                         switch (insr->class_index) {
                               case AlarmEntry_index:
                                    delete_ins = true;
                                    for (j = 0; j < 2; j++) {
                                         if ((ncu_thr_allowed_alentry_list[i].alentry_list[j][0]) &&
                                               (!(strncmp(insr->instanceId,
                                                   ncu_thr_allowed_alentry_list[i].alentry_list[j],INSTANCE_ID_LENGTH)))) {
                                                         delete_ins = false;
                                         }
                                    }
                                    if (delete_ins) {
                                        ins_del = insr;
                                    }
                                    break;
                               default:
                                   break;
                         }
                  }
                  if (ins_del) {
                        MO_unlink(ins_del);
                        ins_del = NULL;
                   }
                  break;
               }
           }
       break;
       default:
           return NULL;
    }
    return NULL;
}
