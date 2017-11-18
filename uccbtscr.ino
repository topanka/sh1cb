#define TSCR_CBOX_LEAD      0x31
#define TSCR_TSCR_LEAD      0x44

#define TSCR_PST_INIT          1
#define TSCR_PST_DATA          2
#define TSCR_PST_CRC           3
#define TSCR_PST_READY        66

#define TSCR_CBOX_PKTLEN      42
#define TSCR_CBOX_PKTLAST     (TSCR_CBOX_PKTLEN-2)

#define TSCR_TSCR_PKTLEN      10
#define TSCR_TSCR_PKTLAST     (TSCR_TSCR_PKTLEN-2)

byte g_wts_commbuf[100]={0};
unsigned long g_wts_sendtime=0;
unsigned long g_wts_commpkt_counter=0;        //total counter of sent packets

byte g_rts_commbuf[100]={0};
int g_rts_state=TSCR_PST_INIT;
unsigned int g_rts_len=0;

int tscr_setup(void)
{
  Serial3.begin(115200);
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
  tscr_comm_pack1((byte*)&g_sh1_m1rpm,sizeof(g_sh1_m1rpm),g_wts_commbuf,len);    //1:39
  tscr_comm_pack1((byte*)&g_sh1_m2rpm,sizeof(g_sh1_m2rpm),g_wts_commbuf,len);    //1:41
  
  crc8=getCRC(g_wts_commbuf,*len);
  
//Serial.print("crc8 ");  
//Serial.println(crc8);  
  
  tscr_comm_pack1((byte*)&crc8,sizeof(crc8),g_wts_commbuf,len);    //1:42
  
//42 byte long  
  
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
                    int *touchx,
                    int *touchy)
{
  unsigned int l;
  
  l=1;
  tscr_comm_unpack1((unsigned char *)loop_cps,sizeof(unsigned long),buf,&l);
  tscr_comm_unpack1((unsigned char *)touchx,sizeof(int),buf,&l);
  tscr_comm_unpack1((unsigned char *)touchy,sizeof(int),buf,&l);

  return(0);
}

int tscr_comm_recv(void)
{
  int ret;

  ret=tscr_comm_read(&g_rts_state,g_rts_commbuf,&g_rts_len);
  
//                  Serial.println(ret);
  
  if(ret == TSCR_PST_READY) {
    tscr_comm_unpackuccb(g_rts_commbuf,g_rts_len,
                    &g_tscr_loopcps,
                    &g_tscr_touchx,
                    &g_tscr_touchy);

                    Serial.println(g_tscr_touchx);
  }

  return(0);
}

int tscr_comm(void)
{
  tscr_comm_send();
  tscr_comm_recv();
}

