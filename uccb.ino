#include <Servo.h> 
#include <uccbst.h>
#include <uccbar.h>
#include <uccbbtn.h>
#include <uccbcrc.h>
#include <sh1tmr.h>
#include <fim5360.h>

#include <LiquidCrystal.h>
#include <Keypad.h>
#include <RunningAverage.h>
#include <umpmega.h>

#define UCCB_VERSION        "1.3.6"

//battery
#define UCCB_BATTV_PORT          A5
//y=ax+b; a=0.0125511, b=-1.96401
int g_battV=-1;
long g_battV_a=1255;
long g_battV_b=-196401;
//lipo discharch voltage 7.4V
int g_battV_min=746;
//lipo critical discharch voltage 7.3V
int g_battV_crit=738;
//lipo shutdown voltage 7.2V
int g_battV_shutdown=734;
MYTMR g_tmr_battV_shutdown={0};
MYTMR g_tmr_battV={0};
SMAR g_smar_battV={0};

#define UCCB_ADJPS_ON_UMPPORT     0       //PE2
#define UCCB_ADJPS_ON_ON          LOW
#define UCCB_ADJPS_ON_OFF         HIGH

#define UCCB_ADJPS_LEVEL_UMPPORT  3       //PH2
#define UCCB_ADJPS_LEVEL_HIGH     LOW
#define UCCB_ADJPS_LEVEL_LOW      HIGH

int g_adjps_on=0;
int g_adjps_level=-1;
#define UCCB_ADJV_PORT           A1
SMAR g_smar_ADJV={0};
int g_adjV=-1;

#define UCCB_FS_SRV_PWR_UMPPORT  4       //PH7
#define UCCB_FS_SRV_PWR_ON       LOW
#define UCCB_FS_SRV_PWR_OFF      HIGH

int g_fs_servo_on=0;

#define UCCB_3_3V_PORT          A8
SMAR g_smar_3_3V={0};
int g_3_3V=-1;

#define UCCB_5_0V_PORT          A9
SMAR g_smar_5_0V={0};
int g_5_0V=-1;

#define UCCB_BATTA_PORT         A0
int g_battA=-1;
MYTMR g_tmr_battA={0};
SMAR g_smar_battA={0};

#define UCCB_SHUTDOWN_PORT      27

#define UCCB_BATTSTATE_PORT     49

uint8_t g_shutdown_state=0;
MYTMR g_tmr_SD={0};

//lcd
LiquidCrystal lcd(12,11,5,4,3,2);

//joy
//#define UCCB_TS_X             A4
//#define UCCB_TS_Y             A0
#define UCCB_TS_X               MCP3008_CH0
#define UCCB_TS_Y               MCP3008_CH1

int g_tsX=-1;
int g_tsY=-1;

//#define UCCB_FS_X             A3
//#define UCCB_FS_Y             A2
//#define UCCB_FS_Z             A1
#define UCCB_FS_X             MCP3008_CH4
#define UCCB_FS_Y             MCP3008_CH3
#define UCCB_FS_Z             MCP3008_CH2

#define UCCB_FS_BTN           6

int g_fsX=-1;
int g_fsY=-1;
int g_fsZ=-1;
int g_fsBS=-1;
int g_fsBE=-1;

int g_fs_minX=0;
int g_fs_minY=0;
int g_fs_minZ=0;
int g_fs_maxX=0;
int g_fs_maxY=0;
int g_fs_maxZ=0;
int g_fs_cenX=0;
int g_fs_cenY=0;
int g_fs_cenZ=0;
int g_fs_cenX0=-1;
int g_fs_cenY0=-1;
int g_fs_cenZ0=-1;

SMAR g_smar_fsX={0};
SMAR g_smar_fsY={0};
SMAR g_smar_fsZ={0};

int g_ts_minX=0;
int g_ts_minY=0;
int g_ts_maxX=0;
int g_ts_maxY=0;
int g_ts_cenX=0;
int g_ts_cenY=0;
int g_ts_cenX0=-1;
int g_ts_cenY0=-1;

SMAR g_smar_tsX={0};
SMAR g_smar_tsY={0};

UCCBBTN g_btn_fs={0};

int g_e_m1s=0;
int g_e_m2s=0;
int g_e_rdd=0;
int g_e_tsx=0;
int g_e_tsy=0;

//10p switch
#define UCCB_SW10P_PORT       A6
int g_sw10p=-1;
MYTMR g_tmr_sw10p={0};
SMAR g_smar_sw10p={0};

//6p button
#define UCCB_B6P_PORT         A7
int g_b6p=-1;
int g_b6pBS=-1;
int g_b6pBE=-1;
MYTMR g_tmr_b6p={0};
SMAR g_smar_b6p={0};
UCCBBTN g_btn_b6p={0};

//keypad
char g_key;

//general
unsigned long g_millis=0;
unsigned long g_loop_cnt=0;         //loop counter
unsigned long g_loop_cps=0;         //loop counter per sec
unsigned long g_loop_ct=0;
unsigned char g_shiptobeready=0;

//SH1

//battery
//y=ax+b; a=0.011548, b=-1.05552
int g_sh1_battV=-1;
long g_sh1_battV_a=1155;
long g_sh1_battV_b=-105552;
//lipo discharch voltage 7.4V
int g_sh1_battV_min=733;

//y=ax+b; a=0.0378846, b=-18.8018
int g_sh1_battA=-1;
long g_sh1_battA_a=3788;
long g_sh1_battA_b=-1880180;

//comm
unsigned long g_sh1_w_commpkt_counter=0;
uint16_t g_sh1_lost_cont=0;
uint16_t g_sh1_lost_tot=0;

//md
unsigned int g_sh1_m1c=0;
unsigned int g_sh1_m2c=0;
uint16_t g_sh1_m1rpm=0;
uint16_t g_sh1_m2rpm=0;
int8_t g_sh1_m1dir=0;
int8_t g_sh1_m2dir=0;
int8_t g_sh1_m1on=1;
int8_t g_sh1_m2on=1;

int g_sh1_poslight=UCCB_PL_OFF;
int g_sh1_maxmspeed=UCCB_MD_MAXSPEED;
int g_sh1_rotscale=0;
float g_sh1_rotcurve=1.0;

int g_sh1_temperature=-1;

unsigned long g_scnt=0;

//touch screen

unsigned long g_tscr_loopcps=0;
int g_tscr_touchx=-1;
int g_tscr_touchy=-1;

// finger scanner

int g_master_auth=0;
int g_operator_auth=0;
char *g_master_password=(char*)"6661";
MYTMR g_tmr_finscan={0};
int g_finscan_pos=1500;
Servo g_servo_finscan;

