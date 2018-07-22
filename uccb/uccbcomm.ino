#define UCCB_COMMMODE_S2R     11

byte g_w_commbuf[100]={0};
unsigned long g_w_sendtime=0;
unsigned long g_w_commpkt_counter=0;        //total counter of sent packets

byte g_r_commbuf[100]={0};
int g_r_state=UCCB_PST_INIT;
unsigned int g_r_len=0;
byte g_commmode=1;
unsigned long g_commmode_t=0;

int comm_setup(void)
{
//  Serial2.begin(9600);
  Serial2.begin(19200);
}

int comm_pack1(byte *d, uint16_t l, byte *buf, uint16_t *len)
{
  if((*len+l) > sizeof(g_w_commbuf)) return(-1);
  memcpy((void*)(buf+*len),(void*)d,l);
  (*len)+=l;
  return(0);
}

int comm_packuccb(int fsBE, int b6pBE, uint16_t *len)
{
  byte lead=UCCB_CBOX_LEAD;
  byte crc8;
  int stb;
  
  *len=0;

  eval_md_speed(&g_e_m1s,&g_e_m2s,&g_e_rdd);
  tscr_md_speed(&g_e_m1s,&g_e_m2s,&g_e_rdd);
  eval_ts_pos(&g_e_tsx,&g_e_tsy);
  stb=g_sw10p;
  if(g_sh1_m1on == 1) stb|=UCCB_ST_M1;
  if(g_sh1_m2on == 1) stb|=UCCB_ST_M2;
  if(g_sh1_poslight == UCCB_PL_ON) stb|=(UCCB_PL_ON<<UCCB_PL_STPOS);
  if(g_sh1_poslight == UCCB_PL_BLINK) stb|=(UCCB_PL_BLINK<<UCCB_PL_STPOS);
  
  comm_pack1((byte*)&lead,sizeof(lead),g_w_commbuf,len);      //1:1
  comm_pack1((byte*)&g_w_commpkt_counter,sizeof(g_w_commpkt_counter),g_w_commbuf,len);    //4:5-32
  comm_pack1((byte*)&g_battV,sizeof(g_battV),g_w_commbuf,len);    //2:7-12
  comm_pack1((byte*)&g_tsX,sizeof(g_tsX),g_w_commbuf,len);    //2:9-12  
  comm_pack1((byte*)&g_tsY,sizeof(g_tsY),g_w_commbuf,len);    //2:11-12
  comm_pack1((byte*)&g_fsX,sizeof(g_fsX),g_w_commbuf,len);    //2:13-12
  comm_pack1((byte*)&g_fsY,sizeof(g_fsY),g_w_commbuf,len);    //2:15-12
  comm_pack1((byte*)&g_fsZ,sizeof(g_fsZ),g_w_commbuf,len);    //2:17-12
  comm_pack1((byte*)&g_fsBS,sizeof(g_fsBS),g_w_commbuf,len);    //2:19-4
  comm_pack1((byte*)&fsBE,sizeof(fsBE),g_w_commbuf,len);    //2:21-8
  comm_pack1((byte*)&stb,sizeof(stb),g_w_commbuf,len);    //2:23-4
  comm_pack1((byte*)&g_b6pBS,sizeof(g_b6pBS),g_w_commbuf,len);    //2:25-4
  comm_pack1((byte*)&b6pBE,sizeof(b6pBE),g_w_commbuf,len);    //2:27-8
  comm_pack1((byte*)&g_e_m1s,sizeof(g_e_m1s),g_w_commbuf,len);    //2:29-1+9
  comm_pack1((byte*)&g_e_m2s,sizeof(g_e_m2s),g_w_commbuf,len);    //2:31-1+9
  comm_pack1((byte*)&g_e_rdd,sizeof(g_e_rdd),g_w_commbuf,len);    //2:33-1+7
  comm_pack1((byte*)&g_e_tsx,sizeof(g_e_tsx),g_w_commbuf,len);    //2:35-1+7
  comm_pack1((byte*)&g_e_tsy,sizeof(g_e_tsy),g_w_commbuf,len);    //2:37-1+7
  comm_pack1((byte*)&g_commmode,sizeof(g_commmode),g_w_commbuf,len);    //1:38
  
//memset((void*)(g_w_commbuf+1),0,*len);
  
  crc8=getCRC(g_w_commbuf,*len);
  
//Serial.print("crc8 ");  
//Serial.println(crc8);  
  
  comm_pack1((byte*)&crc8,sizeof(crc8),g_w_commbuf,len);    //1:39
  
//39 byte long  
  
  return(0);
}

int comm_send(void)
{
  unsigned int len;
  static int l_b6pBE=BTN_NOP;
  static int l_fsBE=BTN_NOP;

  g_w_commpkt_counter++;
  
  if(g_b6pBE != BTN_NOP) l_b6pBE=g_b6pBE;
  if(g_fsBE != BTN_NOP) l_fsBE=g_fsBE;
  
  if((g_millis < g_w_sendtime+35) &&     //19200 RF test
//  if((g_millis < g_w_sendtime+25) &&     //19200 RF
//  if((g_millis < g_w_sendtime+50) &&     //9600 RF
//  if((g_millis < g_w_sendtime+55) && 
     (l_b6pBE == BTN_NOP) &&
     (l_fsBE == BTN_NOP)) return(0);
  
  if(g_commmode == 0) {
    if((g_millis-g_commmode_t) > 500) {
      if(g_sh1_lost_cont < 99) g_sh1_lost_cont++;
      if(g_sh1_lost_tot < 9999) g_sh1_lost_tot++;
      g_commmode=1;
    }
//    g_shiptobeready=1;
//    if((g_millis-g_commmode_t) > 1500) g_commmode=1;
    return(0);
  }
  
//  g_w_commpkt_counter++;
  
  g_commmode++;
  g_commmode%=UCCB_COMMMODE_S2R;
    
  g_w_sendtime=g_millis;
  comm_packuccb(l_fsBE,l_b6pBE,&len);
  Serial2.write((byte*)&g_w_commbuf[0],len);
  if(g_commmode == 0) {
    g_commmode_t=g_millis;
  }
  
  l_b6pBE=BTN_NOP;
  l_fsBE=BTN_NOP;

  g_scnt=g_w_commpkt_counter;
  g_w_commpkt_counter=0;

  return(1);
}

int comm_read(int *state, unsigned char *buf, unsigned int *len)
{
  int rval=-1,ret,nr=0;
  unsigned char c1;
  unsigned char crc8;

  while(Serial2.available()) {
    c1=(unsigned char)Serial2.read();
    
//  Serial.println("ship serial avail");
  
    if(++nr >= 100) break;
    switch(*state) {
      case UCCB_PST_INIT:
      case UCCB_PST_READY:
/*      
  Serial.print(*len);
  Serial.print(" ");
  Serial.print(c1);
  Serial.println(" init/ready");
*/  
        if(c1 == UCCB_SHIP_LEAD) {
          *len=0;
          buf[*len]=c1;
          *state=UCCB_PST_DATA;
	} else {
          *state=UCCB_PST_INIT;
        }
	break;
      case UCCB_PST_DATA:
/*      
  Serial.print(*len);
  Serial.print(" ");
  Serial.print(c1);
  Serial.println(" data");
*/  
        (*len)++;
        buf[*len]=c1;
        if(*len == UCCB_SHIP_PKTLAST) {
          *state=UCCB_PST_CRC;
        }
        break;
      case UCCB_PST_CRC:
/*      
  Serial.print(*len);
  Serial.print(" ");
  Serial.print(c1);
  Serial.println(" crc1");
*/  
        if(*len != UCCB_SHIP_PKTLAST) {
          *state=UCCB_PST_INIT;
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
          *state=UCCB_PST_INIT;
          break;
        }
        *state=UCCB_PST_READY;
        return(UCCB_PST_READY);
      default:
        *state=UCCB_PST_INIT;
        break;  
    }
  }
  
  return(rval);
}

int comm_unpack1(unsigned char *d, unsigned int l, unsigned char *buf, unsigned int *len)
{
  memcpy((void*)d,(void*)(buf+*len),l);
  (*len)+=l;
  return(0);
}

int comm_unpackuccb(unsigned char *buf, unsigned int len, 
                    unsigned long *commpkt_counter,
                    int *battV,
                    int *battA,
                    unsigned int *m1c,
                    unsigned int *m2c,
                    int16_t *m1rpm,
                    int16_t *m2rpm,
                    int *temp)
{
  unsigned int l;
  
  l=1;
  comm_unpack1((unsigned char *)commpkt_counter,sizeof(unsigned long),buf,&l);
  comm_unpack1((unsigned char *)battV,sizeof(int),buf,&l);
  comm_unpack1((unsigned char *)battA,sizeof(int),buf,&l);
  comm_unpack1((unsigned char *)m1c,sizeof(unsigned int),buf,&l);
  comm_unpack1((unsigned char *)m2c,sizeof(unsigned int),buf,&l);
  comm_unpack1((unsigned char *)m1rpm,sizeof(uint16_t),buf,&l);
  comm_unpack1((unsigned char *)m2rpm,sizeof(uint16_t),buf,&l);
  comm_unpack1((unsigned char *)temp,sizeof(int),buf,&l);

  return(0);
}

int comm_recv(void)
{
  int ret;
  int16_t rpm1,rpm2;

//csakhogy ne olvasson soha  
//g_commmode=1;

  if(g_commmode != 0) return(0);
  
//delay(50);  
  
  ret=comm_read(&g_r_state,g_r_commbuf,&g_r_len);
  
  if(ret == UCCB_PST_READY) {
    comm_unpackuccb(g_r_commbuf,g_r_len,
                    &g_sh1_w_commpkt_counter,
                    &g_sh1_battV,
                    &g_sh1_battA,
                    &g_sh1_m1c,
                    &g_sh1_m2c,
                    &rpm1,
                    &rpm2,
                    &g_sh1_temperature);
    g_shiptobeready=1;
    g_commmode=1;
    g_sh1_lost_cont=0;
    g_sh1_m1rpm=abs(rpm1);
    if(rpm1 >= 0)  g_sh1_m1dir=1;
    else g_sh1_m1dir=-1;
    g_sh1_m2rpm=abs(rpm2);
    if(rpm2 >= 0)  g_sh1_m2dir=1;
    else g_sh1_m2dir=-1;
  }
//  g_commmode=1;

  return(0);
}

