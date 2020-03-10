#ifndef EEM_H
#define EEM_H

#define SOCKADDR(p) ((struct sockaddr *)p)
#define EEM_MTU 1536
#define EEM_TIMEOUT 10
#define EEM_LOST_TIMEOUT 60
#define EEM_READ_TIMEOUT 1
#define EEM_SCAN_PERIOD 90
#define POLL_LEN 9
#define REQUEST_LEN (EEM_MTU - 18)
#define IDLEN 5
#define NAMELEN 32
#define MAX_SEND_COUNT 2
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
#define CSU_ID              "0000"
#define BATTERY_GROUP_ID    "0300"
#define RECTIFIER_GROUP_ID  "0200"
#define BATTERY_UNIT_ID     "0301"
#define LVD_GROUP_ID        "0700"
#define SOLAR_GROUP_ID	    "2600"
#define SOLAR_PREFIX	    "26"
#define MAIN_LD_CONID       1
#define PRIO_LD_CONID       2
#define BLVD_LD_CONID       3
#define MAX_TEMP_SENS       4     /* Maximum number of temperature sensors */
#define LVD_RECONNECT_TIME  2 //Two minutes default setting of reconnect delay
#define RECONNECT_VOLT_MIN  40
#define RECONNECT_VOLT_MAX  60
#define DISCONNECT_VOLT_MIN 40
#define DISCONNECT_VOLT_MAX 60
#define DISCONNECT_TIME_MIN 1
#define DISCONNECT_TIME_MAX 1000

#define BATT_TEST_START 10
#define BATT_TEST_STOP  12
#define BATT_TEST_AUTO  16
#define NCU_INV_LEN     15
#define NCU_SNMP_POLL_INTERVAL  2
#define NCU_NUM_BATT_TEMP_SENS  3

#define EEM_DEFAULT_LVD1 45.
#define EEM_DEFAULT_LVD2 44.
#define EEM_DEFAULT_BVLD 43.5
#define EEM_DEFAULT_RECONNECT 50.
#define EEM_LVD_UNIT_ID3    "0703"
#define EEM_SYSTEM_BATTERY_PROFILE_ID "SystemBatteryProfile"
#define NCU_NUM_CONS_DCM    3
#define UNDERVOLT1_DIFF     1.  // Undervoltage alarm shall activate 1 V above LVD1
#define EEMIDX_UNDERVOLT1   1
#define EEMIDX_UNDERVOLT2   3

#define ALARM_SPECIFIC_PROBLEM_LENGTH 129

#define CID_ENUM(x) (3*x - 3)
#define PLD(X) e->ld[X]
#define IS_LLVD(X) (PLD(X)) ? MO_private(PLD(X)) : NULL
#define DISC_VOLT_DEP(x) (!isnanf(x) && (x < RECONNECT_VOLT_MIN || x > DISCONNECT_VOLT_MAX))
#define DISC_TIME_DEP(x) (!isnanf(x) && (x < DISCONNECT_TIME_MIN || x > DISCONNECT_TIME_MAX))
#define REC_VOLT_DEP(x) (!isnanf(x) && (x < RECONNECT_VOLT_MIN || x > RECONNECT_VOLT_MAX))
#define IS_IN_RANGE(x, a, b) ((((x) >= (a)) && ((x) <= (b))) ? true : false)

#define PWR_NCU_THR_TEMPALARMLOW (-40.)
#define PWR_NCU_THR_TEMPALARMLOW_SMART (PWR_NCU_THR_TEMPALARMLOW)
#define PWR_NCU_THR_TEMPALARMHIGH (90.)
#define PWR_NCU_THR_TEMPALARMHIGH_SMART (PWR_NCU_THR_TEMPALARMHIGH)
#define PWR_NCU_THR_TEMPDISCHIGH (60.)
#define PWR_NCU_THR_TEMPDISCHIGH_SMART (PWR_NCU_THR_TEMPDISCHIGH)
#define PWR_NCU_THR_TEMPRECONHIGH (PWR_NCU_THR_TEMPDISCHIGH - 5)
#define PWR_NCU_THR_TEMPRECONHIGH_SMART (PWR_NCU_THR_TEMPDISCHIGH_SMART - 5)
#define PWR_NCU_THR_HIGHSYSTEMVOLTAGE 58.5
#define PWR_NCU_THR_HIGHSYSTEMVOLTAGE_SMART (PWR_NCU_THR_HIGHSYSTEMVOLTAGE)
#define PWR_NCU_THR_BATTTEMPHIGH (55.)
#define PWR_NCU_THR_BATTTEMPLOW (-40.)

typedef void eem_callback_t(char *, size_t, void *);

typedef enum {
    EEM_INACTIVE,
    EEM_CONNECTING,
    EEM_CONNECTED
} eem_state_t;

typedef enum {
    EEM_VOLTAGE_DM = 0,
    EEM_TIME_DM = 1
} eem_dm_method_t;

typedef enum {
    NCUSNMP_RECTNUM,
    NCUSNMP_RECTKEY,
    NCUSNMP_RECTID,
    NCUSNMP_MULTIRQ,
    NCUSNMP_INVALID = 0xFF
} ncu_snmp_state_t;

struct eemdata {
    float *ai_value;
    float *ao_value;
    uint8_t *di_value;
    uint8_t *do_value;
};

struct eemr {
    struct list_head list;
    size_t request_len;
    char request[REQUEST_LEN];
    int send_count;
    eem_callback_t *user_callback;
    void *user_data;
};

#ifdef EEMPHY
struct eemphy {
    struct list_head list;
    struct eem *eem;
    char id[IDLEN];
    char group[4];
    char subgroup[4];
    char product_number[24];
    char serial_number[16];
    char HW_revision[8];
    char SW_revision[8];
};

struct eemphr {
    struct list_head list;
    struct eemphy *eemphy;
};
#endif

struct eem {
    struct list_head device;
    struct list_head physical;
    struct list_head queue;
    struct pwr *pwr;
    ip_ssaddr_t dest;
    MO_instance *ld[2];
    struct bufferevent *bev;
    struct event *timeout_event;
    struct event *scan_event;
    struct event *event;
    struct event *snmp_event;
    char name[NAMELEN];
    char cc_id[2];
    eem_state_t state;
    char SWrevision[NCU_INV_LEN];
    char ProductModel[NCU_INV_LEN];
    char SerialNum[NCU_INV_LEN];
    bool debug;
    uint8_t tmout_cnt;
    bool rb_rq_loop;        /* RB request loop active */
    struct snmpget_multi *sm;
    ncu_snmp_state_t snmp_status;
};

struct eem_unit {
    uint8_t ccid;
};

struct eem_proto {
    struct eem *e;
};

struct pwr_alarm_defSpecificProblem_list {
    PowerSystemType_t type;
    struct alarm_defSpecificProblem *list;
    size_t size;
};

struct alarm_defSpecificProblem {
    class_index_t class_index;
    const char instanceId[INSTANCE_ID_LENGTH];
    const char specificProblem[ALARM_SPECIFIC_PROBLEM_LENGTH];
};

struct thr_allowed_alentry_list {
    class_index_t par_class_index;
    const char instanceId[INSTANCE_ID_LENGTH];
    const char alentry_list[2][INSTANCE_ID_LENGTH];
};

typedef enum {
    NCUSYS_SNMPMODEL,
    NCUSYS_SNMPFWVER,
    NCUSYS_SNMPNAME,
    NCUSYS_SNMPSERNM,
    NCUSYS_NUMSNMPREQ
} ncu_sys_snmp_t;

typedef enum {
    NCURECT_SNMPPROD,
    NCURECT_SNMPHWVER,
    NCURECT_SNMPSWVER,
    NCURECT_SNMPSERNUM,
    NCURECT_SNMPIDENT,
    NCURECT_NUMSNMPREQ
} ncu_rect_snmp_param_t;

struct eem;
extern const char *eem_open(struct pwr *);
extern struct eemdev *eemdev_find_ins(MO_instance *);
extern struct eemdata *eemdata_find_ins(MO_instance *);
extern struct eemdata *eemdata_find(struct pwr *, const char *);
extern struct eemdata *eemdata_find_dep(const char *);
extern struct eemdev *eemdev_find(struct eem *, const char *);
extern struct eem *eem_find_parent(MO_instance *);
extern bool eem_get_bat(void *, float *, float *, float *, float *, float *);
extern void eem_set_equalize_charge_stop_delay_time(struct pwr *, float);
extern void eem_set_equalize_charge_stop_current(struct pwr *, float);
extern void eem_set_cyclic_equalize_charge_interval(struct pwr *, float);
extern void eem_set_boost_charge(struct pwr *, bool);
//extern void eem_set_lvd(struct pwr *, struct pwr_out *, float, float);
extern void eem_set_contactors(struct pwr *, uint8_t, uint8_t);
extern void eem_set_eco_mode(struct pwr *, bool);
extern void eem_rectifier_enable(struct pwr *, struct pwr_in *, bool);
extern void eem_command(struct pwr *, const char *);
extern const char *eem_restart(struct pwr *);
const char *eem_set_config_float(struct pwr *, pwr_config_float_t , float);
extern bool eem_can_set_config_float(pwr_config_float_t type);
float eem_round_config_float(struct pwr *, pwr_config_float_t, float );
extern void eem_debug(struct pwr *, bool);
extern void eem_delete(struct pwr *);
extern void eem_loadD_volt_config_update(struct pwr *, struct loadD *);
extern void eem_loadD_time_config_update(struct pwr *, struct loadD *);
extern bool eem_loadD_check_support(disconnectMethod_t, counter_state_t);
extern const char *eem_loadD_local_set(struct pwr *, struct loadD *);
extern void eem_set_loadD(struct pwr*, uint8_t idx);
extern void eem_set_blvd_dm(struct pwr *);
extern void eem_set_loadD_dm(struct pwr *, uint8_t);
extern const char *eem_handler_load_disc(struct pwr *, struct loadD *,
        MO_instance *);
extern const char *eem_check_BLVD (struct pwr *, MO_instance *);
extern const char *eem_check_LVD (struct pwr *, MO_instance *);
extern void eem_loadD_get_device_config(struct pwr *, struct loadD *);
extern void eem_reconnect(struct eem *);
void eem_thrh_value(struct pwr *, struct thrh *);
#ifdef MEMDEBUG
extern void eem_cleanup(void);
#endif
void eem_bat_test(struct pwr *, bool);
char * eem_get_sys_serial(struct pwr *);
#endif
