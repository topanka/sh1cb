#include "uccbtscr.h"

byte g_wts_commbuf[100]={0};
unsigned long g_wts_sendtime=0;
unsigned long g_wts_commpkt_counter=0;        //total counter of sent packets

byte g_rts_commbuf[100]={0};
int g_rts_state=TSCR_PST_INIT;
unsigned int g_rts_len=0;

int tscr_setup(void)
{
  ump_pinMode(UCCB_TSCR_ON_UMPPORT,OUTPUT);
  if(g_tscr_on == UCCB_TSCR_ON_ON) {
    ump_digitalWrite(UCCB_TSCR_ON_UMPPORT,UCCB_TSCR_ON_ON);
  } else {
    ump_digitalWrite(UCCB_TSCR_ON_UMPPORT,UCCB_TSCR_ON_OFF);
  }
  Serial3.begin(250000);
  return(0);
}

int tscr_comm_pack1(byte *d, uint16_t l, byte *buf, uint16_t *len)
{
  if((*len+l) > sizeof(g_wts_commbuf)) return(-1);
  memcpy((void*)(buf+*len),(void*)d,l);
  (*len)+=l;
  return(0);
}

int tscr_comm_packuccb(int fsBE, int b6pBE, uint16_t *len)
{
  byte lead=TSCR_CBOX_LEAD;
  byte crc8;
  int stb;
  
  *len=0;

  stb=g_sw10p;
  if(g_sh1_m1on == 1) stb|=UCCB_ST_M1;
  if(g_sh1_m2on == 1) stb|=UCCB_ST_M2;
  if(g_sh1_poslight == UCCB_PL_ON) stb|=(UCCB_PL_ON<<UCCB_PL_STPOS);
  if(g_sh1_poslight == UCCB_PL_BLINK) stb|=(UCCB_PL_BLINK<<UCCB_PL_STPOS);

  tscr_comm_pack1((byte*)&lead,sizeof(lead),g_wts_commbuf,len);      //1:1
  tscr_comm_pack1((byte*)&g_wts_commpkt_counter,sizeof(g_wts_commpkt_counter),g_wts_commbuf,len);    //4:5-32
  tscr_comm_pack1((byte*)&g_battV,sizeof(g_battV),g_wts_commbuf,len);    //2:7-12
  tscr_comm_pack1((byte*)&g_tsX,sizeof(g_tsX),g_wts_commbuf,len);    //2:9-12  
  tscr_comm_pack1((byte*)&g_tsY,sizeof(g_tsY),g_wts_commbuf,len);    //2:11-12
  tscr_comm_pack1((byte*)&g_fsX,sizeof(g_fsX),g_wts_commbuf,len);    //2:13-12
  tscr_comm_pack1((byte*)&g_fsY,sizeof(g_fsY),g_wts_commbuf,len);    //2:15-12
  tscr_comm_pack1((byte*)&g_fsZ,sizeof(g_fsZ),g_wts_commbuf,len);    //2:17-12
  tscr_comm_pack1((byte*)&g_fsBS,sizeof(g_fsBS),g_wts_commbuf,len);    //2:19-4
  tscr_comm_pack1((byte*)&fsBE,sizeof(fsBE),g_wts_commbuf,len);    //2:21-8
  tscr_comm_pack1((byte*)&stb,sizeof(stb),g_wts_commbuf,len);    //2:23-4
  tscr_comm_pack1((byte*)&g_b6pBS,sizeof(g_b6pBS),g_wts_commbuf,len);    //2:25-4
  tscr_comm_pack1((byte*)&b6pBE,sizeof(b6pBE),g_wts_commbuf,len);    //2:27-8
  tscr_comm_pack1((byte*)&g_e_m1s,sizeof(g_e_m1s),g_wts_commbuf,len);    //2:29-1+9
  tscr_comm_pack1((byte*)&g_e_m2s,sizeof(g_e_m2s),g_wts_commbuf,len);    //2:31-1+9
  tscr_comm_pack1((byte*)&g_e_rdd,sizeof(g_e_rdd),g_wts_commbuf,len);    //2:33-1+7
  tscr_comm_pack1((byte*)&g_e_tsx,sizeof(g_e_tsx),g_wts_commbuf,len);    //2:35-1+7
  tscr_comm_pack1((byte*)&g_e_tsy,sizeof(g_e_tsy),g_wts_commbuf,len);    //2:37-1+7
  tscr_comm_pack1((byte*)&g_sh1_m1rpm,sizeof(g_sh1_m1rpm),g_wts_commbuf,len);    //2:39
  tscr_comm_pack1((byte*)&g_sh1_m2rpm,sizeof(g_sh1_m2rpm),g_wts_commbuf,len);    //2:41
  tscr_comm_pack1((byte*)&g_tscr_takeover,sizeof(g_tscr_takeover),g_wts_commbuf,len);    //1:42
  
  crc8=getCRC(g_wts_commbuf,*len);
  
//Serial.print("crc8 ");  
//Serial.println(crc8);  
  
  tscr_comm_pack1((byte*)&crc8,sizeof(crc8),g_wts_commbuf,len);    //1:43
  
//43 byte long  
  
  return(0);
}

int tscr_comm_send(void)
{
  unsigned int len;
  static int l_b6pBE=BTN_NOP;
  static int l_fsBE=BTN_NOP;

  g_wts_commpkt_counter++;
  
  if(g_b6pBE != BTN_NOP) l_b6pBE=g_b6pBE;
  if(g_fsBE != BTN_NOP) l_fsBE=g_fsBE;
  
  if((g_millis < g_wts_sendtime+50) &&
     (l_b6pBE == BTN_NOP) &&
     (l_fsBE == BTN_NOP)) return(0);

  g_wts_sendtime=g_millis;
  tscr_comm_packuccb(l_fsBE,l_b6pBE,&len);
  Serial3.write((byte*)&g_wts_commbuf[0],len);
  
  l_b6pBE=BTN_NOP;
  l_fsBE=BTN_NOP;

  g_wts_commpkt_counter=0;

  return(1);
}

int tscr_comm_read(int *state, unsigned char *buf, unsigned int *len)
{
  int rval=-1,ret,nr=0;
  unsigned char c1;
  unsigned char crc8;

  while(Serial3.available()) {
    c1=(unsigned char)Serial3.read();
    if(++nr >= 100) break;
    switch(*state) {
      case TSCR_PST_INIT:
      case TSCR_PST_READY:
/*      
  Serial.print(*len);
  Serial.print(" ");
  Serial.print(c1);
  Serial.println(" init/ready");
*/  
        if(c1 == TSCR_TSCR_LEAD) {
          *len=0;
          buf[*len]=c1;
          *state=TSCR_PST_DATA;
        } else {
          *state=TSCR_PST_INIT;
        }
        break;
      case TSCR_PST_DATA:
/*      
  Serial.print(*len);
  Serial.print(" ");
  Serial.print(c1);
  Serial.println(" data");
*/  
        (*len)++;
        buf[*len]=c1;
        if(*len == TSCR_TSCR_PKTLAST) {
          *state=TSCR_PST_CRC;
        }
        break;
      case TSCR_PST_CRC:
/*      
  Serial.print(*len);
  Serial.print(" ");
  Serial.print(c1);
  Serial.println(" crc1");
*/  
        if(*len != TSCR_TSCR_PKTLAST) {
          *state=TSCR_PST_INIT;
          break;
        }
        (*len)++;
        crc8=getCRC(buf,*len);
/*        
  Serial.print(c1);
  Serial.print(" ");
  Serial.println(crc8);
*/  
        if(crc8 != c1) {
          *state=TSCR_PST_INIT;
          break;
        }
        *state=TSCR_PST_READY;
        return(TSCR_PST_READY);
      default:
        *state=TSCR_PST_INIT;
        break;  
    }
  }
  
  return(rval);
}

int tscr_comm_unpack1(unsigned char *d, unsigned int l, unsigned char *buf, unsigned int *len)
{
  memcpy((void*)d,(void*)(buf+*len),l);
  (*len)+=l;
  return(0);
}

int tscr_comm_unpackuccb(unsigned char *buf, unsigned int len, 
                    unsigned long *loop_cps,
                    int *touchx, int *touchy,
                    int *power, int *rudder,
                    int *poslight)
{
  unsigned int l;
  
  l=1;
  tscr_comm_unpack1((unsigned char *)loop_cps,sizeof(unsigned long),buf,&l);
  tscr_comm_unpack1((unsigned char *)touchx,sizeof(int),buf,&l);
  tscr_comm_unpack1((unsigned char *)touchy,sizeof(int),buf,&l);
  tscr_comm_unpack1((unsigned char *)power,sizeof(int),buf,&l);
  tscr_comm_unpack1((unsigned char *)rudder,sizeof(int),buf,&l);
  tscr_comm_unpack1((unsigned char *)poslight,sizeof(int),buf,&l);

  return(0);
}

int tscr_comm_recv(void)
{
  int ret,poslight;

  ret=tscr_comm_read(&g_rts_state,g_rts_commbuf,&g_rts_len);
  
//                  Serial.println(ret);
  
  if(ret == TSCR_PST_READY) {
    tscr_comm_unpackuccb(g_rts_commbuf,g_rts_len,
                    &g_tscr_loopcps,
                    &g_tscr_touchx,
                    &g_tscr_touchy,
                    &g_tscr_power,
                    &g_tscr_rudder,
                    &poslight);

/*
                    Serial.print(g_tscr_power);
                    Serial.print(" ");
                    Serial.println(g_tscr_rudder);
*/  

    if(g_sh1_poslight == go_sh1_poslight) {
      if(poslight != g_sh1_poslight) {
        g_sh1_poslight=poslight;
        go_sh1_poslight=poslight;
        if(g_sw10p == 8) {
          dsp_scr_ship3(1);
        }
      }
    } else {
      if(g_sh1_poslight == poslight) {
        go_sh1_poslight=poslight;
      }
    }

  }

  return(0);
}

int tscr_comm(void)
{
  tscr_comm_send();
  tscr_comm_recv();
}

int tscr_md_speed(int *m1s, int *m2s, int *rdd)
{
  g_tscr_takeover=0;
  if((*m1s != 0) || (*m2s != 0) || (*rdd != 0)) return(0);
  if((g_tscr_power == 0) && (g_tscr_rudder == 0)) return(0);
  g_tscr_takeover=1;
//  *m1s=(g_tscr_power*(g_sh1_maxmspeed-UCCB_MD_MINSPEED_FWD))/100;

  if(g_tscr_power > 0) {
    *m1s=UCCB_MD_MINSPEED_FWD+(g_tscr_power*(g_sh1_maxmspeed-UCCB_MD_MINSPEED_FWD))/100;
  } else if(g_tscr_power < 0) {
    *m1s=-UCCB_MD_MINSPEED_FWD+(g_tscr_power*(g_sh1_maxmspeed-UCCB_MD_MINSPEED_FWD))/100;
  } else {
    *m1s=0;
  }

  *m2s=*m1s;
  *rdd=(((100*g_tscr_rudder)/90)*UCCB_RDD_MAXPOS)/100;
  return(0);
}

