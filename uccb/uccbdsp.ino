#define UCCB_DSP_CLB_AC_TXT                  1
#define UCCB_DSP_CLB_AC_CLB                  2
#define UCCB_DSP_CLB_TSCENX_TXT              3
#define UCCB_DSP_CLB_TSCENX_CLB              4
#define UCCB_DSP_CLB_TSMINX_TXT              5
#define UCCB_DSP_CLB_TSMINX_CLB              6
#define UCCB_DSP_CLB_TSMAXX_TXT              7
#define UCCB_DSP_CLB_TSMAXX_CLB              8
#define UCCB_DSP_CLB_TSCENY_TXT              9
#define UCCB_DSP_CLB_TSCENY_CLB              10
#define UCCB_DSP_CLB_TSMINY_TXT              11
#define UCCB_DSP_CLB_TSMINY_CLB              12
#define UCCB_DSP_CLB_TSMAXY_TXT              13
#define UCCB_DSP_CLB_TSMAXY_CLB              14

#define UCCB_DSP_CLB_FSCENX_TXT              15
#define UCCB_DSP_CLB_FSCENX_CLB              16
#define UCCB_DSP_CLB_FSMINX_TXT              17
#define UCCB_DSP_CLB_FSMINX_CLB              18
#define UCCB_DSP_CLB_FSMAXX_TXT              19
#define UCCB_DSP_CLB_FSMAXX_CLB              20
#define UCCB_DSP_CLB_FSCENY_TXT              21
#define UCCB_DSP_CLB_FSCENY_CLB              22
#define UCCB_DSP_CLB_FSMINY_TXT              23
#define UCCB_DSP_CLB_FSMINY_CLB              24
#define UCCB_DSP_CLB_FSMAXY_TXT              25
#define UCCB_DSP_CLB_FSMAXY_CLB              26
#define UCCB_DSP_CLB_FSCENZ_TXT              27
#define UCCB_DSP_CLB_FSCENZ_CLB              28
#define UCCB_DSP_CLB_FSMINZ_TXT              29
#define UCCB_DSP_CLB_FSMINZ_CLB              30
#define UCCB_DSP_CLB_FSMAXZ_TXT              31
#define UCCB_DSP_CLB_FSMAXZ_CLB              32

#define UCCB_DSP_CLB_MAX                     33

#define UCCB_DSP_SH3_OI_NUM                  4

#define UCCB_DSP_FINSCAN_OI_NUM              5
#define UCCB_DSP_ADJPS_OI_NUM                2
#define UCCB_DSP_TSCR_OI_NUM                 1

#define UCCB_DSP_TOOLS_OI_NUM                5

byte h1[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b10000,
  0b11111,
  0b11111,
  0b01111
};
byte h2[8] = {
  0b01110,
  0b00100,
  0b01111,
  0b01111,
  0b01111,
  0b11111,
  0b11111,
  0b11111
};
byte h3[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b10011,
  0b11110,
  0b11100,
  0b11000
};

unsigned long g_dsp_lastprinttime=0;
int g_clb_phase=UCCB_DSP_CLB_TSCENX_TXT;
int g_tools_sml=0;

int numPlaces(unsigned long n)
{
  if (n < 10) return 1;
  if (n < 100) return 2;
  if (n < 1000) return 3;
  if (n < 10000) return 4;
  if (n < 100000) return 5;
  if (n < 1000000) return 6;
  if (n < 10000000) return 7;
  if (n < 100000000) return 8;
  if (n < 1000000000) return 9;
  return 10;
}

int printSpaces(long n, int l)
{
  int np;
  
  np=numPlaces(n);
  lcd.print(n);
  for(;l-np > 0;l--) {
    lcd.print(" ");
  }
  return(0);
}

int printZeros(long n, int l)
{
  int np;
  
  np=numPlaces(n);
  for(;l-np > 0;l--) {
    lcd.print("0");
  }
  return(0);
}

void dsp_setup(void)
{
  lcd.begin(20, 4);
  lcd.createChar(0,h1);
  lcd.createChar(1,h2);
  lcd.createChar(2,h3);
}

void dsp_scr_startup(void)
{
  lcd.setCursor(0,0);
  lcd.print("uccb ");
  lcd.print(UCCB_VERSION); 
  lcd.print("  ");
  lcd.write((byte)0);
  lcd.write((byte)1);
  lcd.write((byte)2);
  lcd.print("   ");
  lcd.print(g_sw10p,10);
}

int scr_shutdown_text(void)
{
  if(tmr_do(&g_tmr_SD) == 1) {
    if(g_tmr_SD.cnt%3 != 0) {
      lcd.setCursor(0,3);
      lcd.print(" SHUT DOWN? (C*/Y#) ");
    } else {
      lcd.setCursor(0,3);
      lcd.print("                    ");
    }
  }
  return(0);
}

int dsp_scr_shutdown(void)
{
  if(g_shutdown_state == 1) {
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("                    ");
/*    
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print(" SHUT DOWN? (Y#/C*) ");
*/    
    scr_shutdown_text();
    g_shutdown_state=2;
    return(1);
  } else if(g_shutdown_state == 2) {
    scr_shutdown_text();
    if(g_key == '#') {
      digitalWrite(UCCB_SHUTDOWN_PORT,HIGH);
    } else if(g_key == '*') {
      lcd.setCursor(0,3);
      lcd.print("                    ");
      g_shutdown_state=0;
      return(2);
    }
    return(1);
  }
  return(0);
}
void dsp_scr_main(int force)
{
  static int l_battV=-1;
  static int l_battA=-1;
  static unsigned long l_loop_cps=0;
  static unsigned long l_scnt=0;
  static unsigned long l_slc=0;
  int ret;
  uint8_t x;
  long xx;

  if((ret=dsp_scr_shutdown()) == 1) {
    return;
  } else if(ret == 2) {
    force=1;
  }
  
  if((force == 1) ||
     (l_battV != g_battV) ||
     (l_battA != g_battA) ||
//     (l_scnt != g_scnt) ||
     (l_slc != g_sh1_lost_cont) ||
     (l_loop_cps != g_loop_cps)) {
      
    lcd.setCursor(0,1);
    lcd.print("batt:");
    xx=(g_battV_a*g_battV+g_battV_b)/1000L;
    lcd.print(xx/100,10);
    lcd.print(".");
    xx%=100;
    if(xx < 10) {
      lcd.print("0");
    } 
    lcd.print(xx,10);
    lcd.print("V");
    l_battV=g_battV;

    lcd.print("/");
//    lcd.print(g_battA,10);
//(val-512)/1024*5/185*1000000; 185 mV/A
//526 a kozepe, 2.57V ad 0 terhelesnel
    xx=(g_battA-526)*26.3936;
    lcd.print(xx,10);
    lcd.print("mA ");
    l_battA=g_battA;

//    lcd.print(g_battV);
    
    lcd.setCursor(0,2);
    lcd.print("lcps=");
    lcd.print(g_loop_cps,10);
    lcd.print("  ");
    l_loop_cps=g_loop_cps;
    
    lcd.setCursor(0,3);
    lcd.print("scnt=");
    lcd.print(g_scnt,10);
    lcd.print("  ");
    l_scnt=g_scnt;
    
    if(g_sh1_lost_cont == 0) {
      lcd.setCursor(10,3);
      if(g_shiptobeready == 0) {
        lcd.print("        --");
      } else {
        lcd.print("        Ok");
      }
    } else {
      x=5+numPlaces(g_sh1_lost_cont);
      lcd.setCursor(20-x,3);
      lcd.print("LOST ");
      lcd.print(g_sh1_lost_cont,10);
    }
    l_slc=g_sh1_lost_cont;
  }
}

void dsp_scr_pwr(int force)
{
  static int l_3_3V=-1;
  static int l_5_0V=-1;
  static int l_adjV=-1;
  static int l_b6pBE=-1;
  long xx;

  if((force == 1) ||
     (l_b6pBE != g_b6pBE)) {
    l_b6pBE=g_b6pBE;
  }
  
  if((force == 1) ||
     (l_3_3V != g_3_3V)) {
    lcd.setCursor(0,1);
    lcd.print("3.3V: ");
    xx=(1000L*g_3_3V)/1024L;
    lcd.print(xx/100,10);
    lcd.print(".");
    xx%=100;    
    printZeros(xx,2);
    lcd.print(xx,10);
    lcd.print("V");
    l_3_3V=g_3_3V;
  }
  
  if((force == 1) ||
     (l_5_0V != g_5_0V)) {
    lcd.setCursor(0,2);
    lcd.print("5.0V: ");
    xx=(1000L*g_5_0V)/1024L;
    lcd.print(xx/100,10);
    lcd.print(".");
    xx%=100;    
    printZeros(xx,2);
    lcd.print(xx,10);
    lcd.print("V");
    l_5_0V=g_5_0V;
  }

  if((force == 1) ||
     (l_adjV != g_adjV)) {
    lcd.setCursor(0,3);
    lcd.print("ADJV: ");
    xx=(1000L*g_adjV)/1024L;
    lcd.print(xx/100,10);
    lcd.print(".");
    xx%=100;    
    printZeros(xx,2);
    lcd.print(xx,10);
    lcd.print("V");
    if(g_adjps_on == 1) {
      if(g_adjps_level == UCCB_ADJPS_LEVEL_LOW) {
        lcd.print(" LOW ");
      } else if(g_adjps_level == UCCB_ADJPS_LEVEL_HIGH) {
        lcd.print(" HIGH");
      } else {
        lcd.print(" --- ");
      }
    } else {
      lcd.print(" OFF ");
    }
    l_adjV=g_adjV;
  }
  
}

void dsp_scr_fingerstick(int force)
{
  static int l_fsX=-1;
  static int l_fsY=-1;
  static int l_fsZ=-1;
  static int l_fsBS=-1;
  static int l_fsBE=-1;
  
  if((force == 1) ||
     (l_fsX != g_fsX) ||
     (l_fsY != g_fsY) ||
     (l_fsZ != g_fsZ) ||
     (l_fsBS != g_fsBS) ||
     (l_fsBE != g_fsBE)) {
    lcd.setCursor(0,1);
    lcd.print("f: X=");
//    lcd.print(g_fsX,10);
    printSpaces(g_fsX,4);
    lcd.setCursor(8,1);
    lcd.print(" Y=");
//    lcd.print(g_fsY,10);
    printSpaces(g_fsY,4);
    lcd.setCursor(14,1);
    lcd.print(" Z=");
//    lcd.print(g_fsZ,10);
    printSpaces(g_fsZ,3);
    
    lcd.setCursor(0,2);
    lcd.print("fs_btn: ");
    lcd.print(g_fsBS);
    lcd.setCursor(10,2);
    if(g_fsBE != 0) {
      if(g_fsBE == 1) {
        lcd.print("  ");
      } else {
        lcd.print(g_fsBE);
      }
    }
    
    l_fsX=g_fsX;
    l_fsY=g_fsY;
    l_fsZ=g_fsZ;
    l_fsBS=g_fsBS;
    l_fsBE=g_fsBE;
    
  }
}

void dsp_scr_thumbstick(int force)
{
  static int l_tsX=-1;
  static int l_tsY=-1;
  
  if((force == 1) ||
     (l_tsX != g_tsX) ||
     (l_tsY != g_tsY)) {
    lcd.setCursor(0,3);
    lcd.print("t: X=");
    lcd.print(g_tsX,10);
    lcd.print(" Y=");
    lcd.print(g_tsY,10);
    lcd.print(" ");
    
    l_tsX=g_tsX;
    l_tsY=g_tsY;
    
  }
}

void dsp_scr_b6p(int force)
{
  static int l_b6pBE=-1;
  static int l_b6pBS=-1;
  static int l_b6p=-1;
  
  if((force == 1) ||
     (l_b6pBE != g_b6pBE) ||
     (l_b6pBS != g_b6pBS) ||
     (l_b6p != g_b6p)) {
    lcd.setCursor(0,1);
    lcd.print(g_b6pBS);
    lcd.setCursor(2,1);
    if(g_b6pBE != 0) {
      if(g_b6pBE == 1) {
        lcd.print("  ");
      } else {
        lcd.print(g_b6pBE);
      }
    }
    lcd.setCursor(5,1);
    lcd.print("b6p=");
    lcd.print(g_b6p,10);
    lcd.print("  ");
    lcd.setCursor(10,3);
    l_b6pBE=g_b6pBE;
    l_b6pBS=g_b6pBS;
    l_b6p=g_b6p;
  }
  
}

void dsp_scr_ship1(int force)
{
  static int l_battV=-1;
  static int l_battA=-1;
  static unsigned int l_m1c=0;
  static unsigned int l_m2c=0;
  static int l_temp=-1;
  static int l_sh1_w_commpkt_counter=-1;
  long xx;
  
  if((force == 1) ||
     (l_battV != g_sh1_battV) ||
     (l_battA != g_sh1_battA) ||
     (l_m1c != g_sh1_m1c) ||
     (l_m2c != g_sh1_m2c) ||
     (l_temp != g_sh1_temperature) ||
     (l_sh1_w_commpkt_counter != g_sh1_w_commpkt_counter)) {
    lcd.setCursor(0,1);
    lcd.print("batt:");
    if(g_sh1_battV < 0) {
      lcd.print("0.00");
    } else {
      xx=(g_sh1_battV_a*g_sh1_battV+g_sh1_battV_b)/1000L;
      lcd.print(xx/100,10);
      lcd.print(".");
      xx%=100;
      if(xx < 10) {
        lcd.print("0");
      } 
      lcd.print(xx,10);
    }
    lcd.print("V/");
    l_battV=g_sh1_battV;
    lcd.setCursor(11,1);
    if(g_sh1_battA < 0) {
      lcd.print("0.00");
    } else {
      xx=(g_sh1_battA-512)*55.35;
      lcd.print(xx,10);
    }
    lcd.print("mA   ");
    l_battA=g_sh1_battA;
    lcd.setCursor(0,2);
    lcd.print("mLc=");
    lcd.print(g_sh1_m2c,10);
    if(g_sh1_m2c > 99) lcd.print("  ");
    else lcd.print("   ");
    l_m2c=g_sh1_m2c;
    lcd.setCursor(10,2);
    lcd.print("mRc=");
    lcd.print(g_sh1_m1c,10);
    if(g_sh1_m1c > 99) lcd.print("  ");
    else lcd.print("   ");
    l_m1c=g_sh1_m1c;
    
    lcd.setCursor(0,3);
    lcd.print("cnt=");
    lcd.print(g_sh1_w_commpkt_counter,10);
    lcd.print("   ");
    l_sh1_w_commpkt_counter=g_sh1_w_commpkt_counter;
    
    lcd.setCursor(10,3);
    lcd.print("T=");
    if(g_sh1_temperature < 0) {
      lcd.print(" -- ");
    } else {
      lcd.print(g_sh1_temperature/10,10);
      lcd.print(".");
      lcd.print(g_sh1_temperature%10,10);
      lcd.print((char)223);
      lcd.print("C ");
      l_temp=g_sh1_temperature;
    }
  }
}

void dsp_scr_ship2(int force)
{
  static unsigned int l_m1c=0;
  static unsigned int l_m2c=0;
  static uint16_t l_m1rpm=0;
  static uint16_t l_m2rpm=0;
  static int l_x=0,l_y=0,l_z=0;
  static int8_t l_m1on=1;
  static int8_t l_m2on=1;
  int x,y,z;
  char buf[25];
  
  eval_joy_getpos(&x,&y,&z);
  if(g_b6pBE == 31) {
    g_sh1_m2on=(g_sh1_m2on+1)%2;
  }
  if(g_b6pBE == 61) {
    g_sh1_m1on=(g_sh1_m1on+1)%2;
  }
  if((force == 1) ||
     (l_m1c != g_sh1_m1c) ||
     (l_m2c != g_sh1_m2c) ||
     (l_x != x) ||
     (l_y != y) ||
     (l_z != z) ||
     (l_m1on != g_sh1_m1on) ||
     (l_m2on != g_sh1_m2on) ||
     (l_m1rpm != g_sh1_m1rpm) ||
     (l_m2rpm != g_sh1_m2rpm)) {
      
    l_m1on=g_sh1_m1on;
    l_m2on=g_sh1_m2on;

    lcd.setCursor(10,2);
    lcd.print("mRr=");
    if(g_sh1_m1on == 0) {
      lcd.print("####");
    } else {
      if(g_sh1_m1dir < 0) {
        lcd.print("-");
      }
      lcd.print(g_sh1_m1rpm,10);
      lcd.print("   ");
    }
    l_m1rpm=g_sh1_m1rpm;
    
    lcd.setCursor(0,2);
    lcd.print("mLr=");
    if(g_sh1_m2on == 0) {
      lcd.print("####");
    } else {
      if(g_sh1_m2dir < 0) {
        lcd.print("-");
      }
      lcd.print(g_sh1_m2rpm,10);
      lcd.print("   ");
    }
    l_m2rpm=g_sh1_m2rpm;
    
    lcd.setCursor(10,1);
    lcd.print("mRc=");
    if(g_sh1_m1on == 0) {
      lcd.print("#####");
    } else {
      lcd.print(g_sh1_m1c,10);
      if(g_sh1_m1c > 99) lcd.print("  ");
      else lcd.print("    ");
    }
    l_m1c=g_sh1_m1c;
    
    lcd.setCursor(0,1);
    lcd.print("mLc=");
    if(g_sh1_m2on == 0) {
      lcd.print("#####");
    } else {
      lcd.print(g_sh1_m2c,10);
      if(g_sh1_m2c > 99) lcd.print("  ");
      else lcd.print("    ");
    }
    l_m2c=g_sh1_m2c;
    
    lcd.setCursor(0,3);
    sprintf(buf,"x=% 4d y=% 4d z=% 4d",x,y,z);
    lcd.print(buf);
    l_x=x;
    l_y=y;
    l_z=z;
  }
}

void dsp_scr_clear_line(int row)
{
  lcd.setCursor(0,row);
  lcd.print("                    ");
}

void dsp_scr_master_password(void)
{
  lcd.setCursor(0,1);
  lcd.print("Master password:    ");
  dsp_scr_clear_line(2);
  lcd.setCursor(0,3);
  lcd.print("  (Cancel*/Enter#)  ");
  lcd.setCursor(0,2);
}

void dsp_scr_operator_auth(void)
{
  lcd.setCursor(0,1);
  lcd.print("Press finger ...    ");
  dsp_scr_clear_line(2);
  lcd.setCursor(0,3);
  lcd.print("     (Cancel*)      ");
  lcd.setCursor(0,2);
  tmr_reset(&g_tmr_finscan);
}

void dsp_scr_new_operator(void)
{
  lcd.setCursor(0,1);
  lcd.print("Press finger ...    ");
  lcd.setCursor(0,2);
  lcd.print("Registration        ");
  lcd.setCursor(0,3);
  lcd.print("     (Cancel*)      ");
  lcd.setCursor(0,2);
  tmr_reset(&g_tmr_finscan);
}

void dsp_scr_operator_logoff(void)
{
  lcd.setCursor(0,1);
  lcd.print("Logoff operator?    ");
  dsp_scr_clear_line(2);
  lcd.setCursor(0,3);
  lcd.print("  (Cancel*/Logoff#) ");
  lcd.setCursor(0,2);
}

void dsp_scr_master_logoff(void)
{
  lcd.setCursor(0,1);
  lcd.print("Logoff master?      ");
  dsp_scr_clear_line(2);
  lcd.setCursor(0,3);
  lcd.print("  (Cancel*/Logoff#) ");
  lcd.setCursor(0,2);
}

int match_master_pwd(char *pwd)
{
  int i=0;
  
  while(g_master_password[i] != '\0') {
    if(pwd[i] != g_master_password[i]) return(0);
    i++;
  }
  if(pwd[i] != '\0') return(0);
  return(1);
}

//1. Operator auth
//2. Master auth
//3. New operator
//4. Menu line4
//5. Menu line5

void dsp_scr_fingerscan(int force)
{
  static int l_cli=1;
  static int l_oi=1;
  static int l_key=NO_KEY;
  static int l_mode=0;
  static int l_pwd_pos=0;
  static char l_pwd[10];
  int diff,l2p,fsret;

  if(force == 1) l_mode=0;
  if(l_mode == 1) {
    l_mode=2;
    if(l_oi == 1) {
      if(g_operator_auth == 1) {
        l_mode=0;
        return;
      }
      dsp_scr_operator_auth();
    } else if(l_oi == 2) {
      if(g_master_auth == 1) {
        l_mode=0;
        return;
      }
      dsp_scr_master_password();
      l_pwd_pos=0;
    } else if(l_oi == 3) {
      dsp_scr_new_operator();
    } else if(l_oi == 4) {
      dsp_scr_operator_logoff();
    } else if(l_oi == 5) {
      dsp_scr_master_logoff();
    }
  }
  if(l_mode == 0) {
    finscan_move(0);
    if(g_key == UCCB_KEY_UP) {
      if(l_cli > 1) {
        l_cli--;
        l_oi--;
      } else {
        if(l_oi > 1) {
          l_oi--;
        } else {
          l_oi=UCCB_DSP_FINSCAN_OI_NUM;
          l_cli=3;
        }
      }
    } else if(g_key == UCCB_KEY_DOWN) {
      if(l_cli > 2) {
        if(l_oi >= UCCB_DSP_FINSCAN_OI_NUM) {
          l_cli=1;
          l_oi=1;
        } else {
          l_oi++;
        }
      } else {
        l_oi++;
        l_cli++;
      }
    } else if(g_key == UCCB_KEY_ENTER) {
      l_mode=1;
      if((l_oi == 2) && (g_master_auth != 1)) {
        lcd.blink();
      } else if(l_oi == 3) {
      }
    } else if(g_key == UCCB_KEY_CANCEL) {
      g_tools_sml=0;
    }
  } else {
    if(g_key == UCCB_KEY_CANCEL) {
      l_mode=0;
      l_pwd_pos=0;
      lcd.noBlink();
      finscan_cancel();
    }
  }

  if(l_mode == 2) {
    if(l_oi == 1) {
      if(g_operator_auth == 1) {
        l_mode=0;
        return;
      }
      fsret=finscan_op(FIM5360_IDENT_PORT);
      if(fsret == 1) {
        Serial.println("reg OK");
        lcd.setCursor(0,2);
        lcd.print("         Ok         ");
        l_mode=0;
        lcd.noBlink();
        force=1;
        delay(500);
        g_operator_auth=1;
      } else if(fsret == -1) {
        Serial.println("reg FAILED");
        lcd.setCursor(0,2);
        lcd.print("       Failed       ");
        l_mode=0;
        lcd.noBlink();
        force=1;
        delay(500);
      }
    } else if(l_oi == 2) {
      if(g_master_auth == 1) {
        l_mode=0;
        return;
      }
      if((g_key >= '0') && (g_key <= '9')) {
        if(l_pwd_pos+1 >= sizeof(l_pwd)) return;
        l_pwd[l_pwd_pos]=g_key;
        lcd.print("*");
        l_pwd_pos++;
      } else if(g_key == UCCB_KEY_ENTER) {
        if(l_pwd_pos >= sizeof(l_pwd)) return;
        l_pwd_pos++;
        l_pwd[l_pwd_pos]='\0';
        lcd.setCursor(0,2);
        if(match_master_pwd(l_pwd) == 1) {
          lcd.print("         Ok         ");
          g_master_auth=1;
          } else {
          lcd.print("       Failed       ");
        }
        while(l_pwd_pos > 0) {
          l_pwd_pos--;
          l_pwd[l_pwd_pos]='\0';
        }
        l_pwd_pos=0;
        l_mode=0;
        lcd.noBlink();
        force=1;
        delay(500);
      }
    } else if(l_oi == 3) {
      fsret=finscan_op(FIM5360_REG_PORT);
      if(fsret == 1) {
        Serial.println("reg OK");
        lcd.setCursor(0,2);
        lcd.print("         Ok         ");
        l_mode=0;
        lcd.noBlink();
        force=1;
        delay(500);
      } else if(fsret == -1) {
        Serial.println("reg FAILED");
        lcd.setCursor(0,2);
        lcd.print("       Failed       ");
        l_mode=0;
        lcd.noBlink();
        force=1;
        delay(500);
      }
    } else if(l_oi == 4) {
      if(g_key == UCCB_KEY_ENTER) {
        g_operator_auth=0;
        l_mode=0;
      }
    } else if(l_oi == 5) {
      if(g_key == UCCB_KEY_ENTER) {
        g_master_auth=0;
        l_mode=0;
      }
    }
  }
  
  if((force == 1) ||
     ((l_key != g_key) && (l_mode == 0))) {

    diff=l_cli-l_oi;   
    l2p=1;    
    if((diff+1 >= 1) && (diff+1 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Operator auth:");
      if(g_operator_auth == 1) {
        lcd.print("   Ok");
      } else {
        lcd.print("   --");
      }
      l2p++;
    }
    
    if((diff+2 >= 1) && (diff+2 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Master auth:");
      if(g_master_auth == 1) {
        lcd.print("     Ok");
      } else {
        lcd.print("     --");
      }
      l2p++;
    }
    
    if((diff+3 >= 1) && (diff+3 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("New operator       ");
      l2p++;
    }
    
    if((diff+4 >= 1) && (diff+4 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Operator logoff    ");
      l2p++;
    }
    
    if((diff+5 >= 1) && (diff+5 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Master logoff      ");
      l2p++;
    }
    
    if(l_oi == 1) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 2) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 3) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 4) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 5) {
      lcd.setCursor(0,l_cli);
    }
    l_key=g_key;
  }
}

void dsp_scr_adjps(int force)
{
  static int l_cli=1;
  static int l_oi=1;
  static int l_key=NO_KEY;
  static int l_mode=0;
  int diff,l2p,fsret;

  if(force == 1) l_mode=0;
  if(l_mode == 1) {
    l_mode=2;
    if(l_oi == 1) {
    } else if(l_oi == 2) {
    } else if(l_oi == 3) {
    } else if(l_oi == 4) {
    } else if(l_oi == 5) {
    }
  }
  if(l_mode == 0) {
//    finscan_move(0);
    if(g_key == UCCB_KEY_UP) {
      if(l_cli > 1) {
        l_cli--;
        l_oi--;
      } else {
        if(l_oi > 1) {
          l_oi--;
        } else {
          l_oi=UCCB_DSP_ADJPS_OI_NUM;
          if(l_oi >= 3) l_cli=3;
          else l_cli=UCCB_DSP_ADJPS_OI_NUM;
        }
      }
    } else if(g_key == UCCB_KEY_DOWN) {
      if(l_cli >= UCCB_DSP_ADJPS_OI_NUM) {
        if(l_oi >= UCCB_DSP_ADJPS_OI_NUM) {
          l_cli=1;
          l_oi=1;
        } else {
          l_oi++;
        }
      } else {
        l_oi++;
        l_cli++;
      }
    } else if(g_key == UCCB_KEY_ENTER) {
      l_mode=1;
    } else if(g_key == UCCB_KEY_CANCEL) {
      g_tools_sml=0;
    }
  } else {
    if(g_key == UCCB_KEY_CANCEL) {
      l_mode=0;
    }
  }

  if(l_mode == 2) {
    if(l_oi == 1) {
      if(g_adjps_on == 0) g_adjps_on=1;
      else g_adjps_on=0;
      adjps_power_switch();
      l_mode=0;
      force=1;
    } else if(l_oi == 2) {
      if(g_adjps_level == UCCB_ADJPS_LEVEL_HIGH) g_adjps_level=UCCB_ADJPS_LEVEL_LOW;
      else g_adjps_level=UCCB_ADJPS_LEVEL_HIGH;
      adjps_level_switch();
      l_mode=0;
      force=1;
    } else if(l_oi == 3) {
       l_mode=0;
    } else if(l_oi == 4) {
       l_mode=0;
    }
  }
  
  if((force == 1) ||
     ((l_key != g_key) && (l_mode == 0))) {

    diff=l_cli-l_oi;   
    l2p=1;    
    if((diff+1 >= 1) && (diff+1 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Adj. PS:");
      if(g_adjps_on == 1) {
        lcd.print(" On ");
      } else {
        lcd.print(" Off");
      }
      l2p++;
    }
    
    if((diff+2 >= 1) && (diff+2 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Output level:");
      if(g_adjps_level == UCCB_ADJPS_LEVEL_HIGH) {
        lcd.print(" High");
      } else if(g_adjps_level == UCCB_ADJPS_LEVEL_LOW) {
        lcd.print(" Low ");
      } else {
        lcd.print(" ----");
      }
      l2p++;
    }
    
    if(((diff+3 >= 1) && (diff+3 <= 3)) && (UCCB_DSP_ADJPS_OI_NUM >= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Menu Line 3       ");
      l2p++;
    }
    
    if((diff+4 >= 1) && (diff+4 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Menu Line 4    ");
      l2p++;
    }
    
    if((diff+5 >= 1) && (diff+5 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Menu line5         ");
      l2p++;
    }
    
    if(l_oi == 1) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 2) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 3) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 4) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 5) {
      lcd.setCursor(0,l_cli);
    }
    l_key=g_key;
  }
}

void dsp_scr_tscr(int force)
{
  static int l_cli=1;
  static int l_oi=1;
  static int l_key=NO_KEY;
  static int l_mode=0;
  int diff,l2p;

  if(force == 1) l_mode=0;
  if(l_mode == 1) {
    l_mode=2;
    if(l_oi == 1) {
    } else if(l_oi == 2) {
    } else if(l_oi == 3) {
    } else if(l_oi == 4) {
    } else if(l_oi == 5) {
    }
  }
  if(l_mode == 0) {
    if(g_key == UCCB_KEY_UP) {
      if(l_cli > 1) {
        l_cli--;
        l_oi--;
      } else {
        if(l_oi > 1) {
          l_oi--;
        } else {
          l_oi=UCCB_DSP_TSCR_OI_NUM;
          if(l_oi >= 3) l_cli=3;
          else l_cli=UCCB_DSP_TSCR_OI_NUM;
        }
      }
    } else if(g_key == UCCB_KEY_DOWN) {
      if((l_cli >= 3) || (l_cli >= UCCB_DSP_TSCR_OI_NUM)) {
        if(l_oi >= UCCB_DSP_TSCR_OI_NUM) {
          l_cli=1;
          l_oi=1;
        } else {
          l_oi++;
        }
      } else {
        l_oi++;
        l_cli++;
      }
    } else if(g_key == UCCB_KEY_ENTER) {
      l_mode=1;
    } else if(g_key == UCCB_KEY_CANCEL) {
      g_tools_sml=0;
    }
  } else {
    if(g_key == UCCB_KEY_CANCEL) {
      l_mode=0;
    }
  }

  if(l_mode == 2) {
    if(l_oi == 1) {
      if(g_tscr_on == UCCB_TSCR_ON_OFF) g_tscr_on=UCCB_TSCR_ON_ON;
      else g_tscr_on=UCCB_TSCR_ON_OFF;
      tscr_power_switch();
      l_mode=0;
      force=1;
    } else if(l_oi == 2) {
      l_mode=0;
    } else if(l_oi == 3) {
       l_mode=0;
    } else if(l_oi == 4) {
       l_mode=0;
    }
  }
  
  if((force == 1) ||
     ((l_key != g_key) && (l_mode == 0))) {
    diff=l_cli-l_oi;   
    l2p=1;    
    if((diff+1 >= 1) && (diff+1 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Touch screen:");
      if(g_tscr_on == UCCB_TSCR_ON_ON) {
        lcd.print(" On ");
      } else {
        lcd.print(" Off");
      }
      l2p++;
    }
    
    if(((diff+2 >= 1) && (diff+2 <= 3)) && (UCCB_DSP_TSCR_OI_NUM >= 2)){
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Menu Line 2        ");
      l2p++;
    }

    if(((diff+3 >= 1) && (diff+3 <= 3)) && (UCCB_DSP_TSCR_OI_NUM >= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Menu Line 3        ");
      l2p++;
    }
    
    if((diff+4 >= 1) && (diff+4 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Menu Line 4        ");
      l2p++;
    }
    
    if((diff+5 >= 1) && (diff+5 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Menu line 5        ");
      l2p++;
    }
    
    if(l_oi == 1) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 2) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 3) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 4) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 5) {
      lcd.setCursor(0,l_cli);
    }
    l_key=g_key;
  }
}

void dsp_scr_tools(int force)
{
  static int l_cli=1;
  static int l_oi=1;
  static int l_key=NO_KEY;
  static int l_mode=0;
  int diff,l2p;

  if(force == 1) {
    l_mode=0;
    g_tools_sml=0;
  }
  if((l_mode == 1) && (g_tools_sml == 0)) {
    l_mode=2;
    g_tools_sml=1;
    force=1;
    if(l_oi == 1) {
    } else if(l_oi == 2) {
    } else if(l_oi == 3) {
    } else if(l_oi == 4) {
    }
  }

  if((l_mode == 0) && (g_tools_sml == 0)) {
    if(g_key == UCCB_KEY_UP) {
      if(l_cli > 1) {
        l_cli--;
        l_oi--;
      } else {
        if(l_oi > 1) {
          l_oi--;
        } else {
          l_oi=UCCB_DSP_TOOLS_OI_NUM;
          l_cli=3;
        }
      }
    } else if(g_key == UCCB_KEY_DOWN) {
      if(l_cli > 2) {
        if(l_oi >= UCCB_DSP_TOOLS_OI_NUM) {
          l_cli=1;
          l_oi=1;
        } else {
          l_oi++;
        }
      } else {
        l_oi++;
        l_cli++;
      }
    } else if(g_key == UCCB_KEY_ENTER) {
      l_mode=1;
      if(l_oi == 1) {
      } else if(l_oi == 2) {
        dsp_scr_clear_line(1);
        dsp_scr_clear_line(2);
        dsp_scr_clear_line(3);
      } else if(l_oi == 3) {
        dsp_scr_clear_line(1);
        dsp_scr_clear_line(2);
        dsp_scr_clear_line(3);
      }
    }
  }
  
  if((l_mode == 2) && (g_tools_sml == 1)) {
    if(l_oi == 1) {
      dsp_scr_fingerscan(force);
    } else if(l_oi == 2) {
      dsp_scr_adjps(force);
//      g_tools_sml=0;
    } else if(l_oi == 3) {
      dsp_scr_tscr(force);
//      g_tools_sml=0;
    } else if(l_oi == 4) {
      g_tools_sml=0;
    } else if(l_oi == 5) {
      g_tools_sml=0;
    }
    if(g_tools_sml == 0) {
      l_mode=0;
      force=1;
    }
  }

  if(((force == 1) || ((l_key != g_key) && (l_mode == 0))) &&
     (g_tools_sml == 0)) {

    diff=l_cli-l_oi;   
    l2p=1;    
    if((diff+1 >= 1) && (diff+1 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Finger scanner   ->");
      l2p++;
    }
    
    if((diff+2 >= 1) && (diff+2 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Adj. Pwr. Supply ->");
      l2p++;
    }
    
    if((diff+3 >= 1) && (diff+3 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Touch screen     ->");
      l2p++;
    }
    
    if((diff+4 >= 1) && (diff+4 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Unused2            ");
      l2p++;
    }
    
    if((diff+5 >= 1) && (diff+5 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("Unused3            ");
      l2p++;
    }
    
    if(l_oi == 1) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 2) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 3) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 4) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 5) {
      lcd.setCursor(0,l_cli);
    }
    l_key=g_key;
  }
  
}

//1. position lights
//2. speed
//3. rotation scale
//4. motor on/off

void dsp_scr_ship3(int force)
{
  static int l_sh1_poslight=UCCB_PL_OFF;
  static int l_sh1_maxmspeed=400;
  static int l_sh1_rotscale=0;
  static int8_t l_sh1_m1on=1;
  static int8_t l_sh1_m2on=1;
  static int l_cli=1;
  static int l_oi=1;
  static int l_key=NO_KEY;
  static int l_mode=0;
  int diff,l2p;

  if(force == 1) l_mode=0;
  if(l_mode == 1) l_mode=2;
  if(l_mode == 0) {
    l_sh1_poslight=g_sh1_poslight;
    l_sh1_maxmspeed=g_sh1_maxmspeed;
    l_sh1_rotscale=g_sh1_rotscale;
    l_sh1_m1on=g_sh1_m1on;
    l_sh1_m2on=g_sh1_m2on;
    if(g_key == UCCB_KEY_UP) {
      if(l_cli > 1) {
        l_cli--;
        l_oi--;
      } else {
        if(l_oi > 1) {
          l_oi--;
        } else {
          l_oi=UCCB_DSP_SH3_OI_NUM;
          l_cli=3;
        }
      }
    } else if(g_key == UCCB_KEY_DOWN) {
      if(l_cli > 2) {
        if(l_oi >= UCCB_DSP_SH3_OI_NUM) {
          l_cli=1;
          l_oi=1;
        } else {
          l_oi++;
        }
      } else {
        l_oi++;
        l_cli++;
      }
    } else if(g_key == UCCB_KEY_ENTER) {
      l_mode=1;
      lcd.blink();
      if(l_oi == 2) {
        l_sh1_maxmspeed=-1;
      } else if(l_oi == 3) {
        l_sh1_rotscale=-1;
      }
    }
  } else {
    if(g_key == UCCB_KEY_CANCEL) {
      l_mode=0;
      lcd.noBlink();
    }
  }
  
  if(l_mode == 2) {
    if(l_oi == 1) {
      if(g_key == '0') {
        l_sh1_poslight=UCCB_PL_OFF;
      } else if(g_key == '1') {
        l_sh1_poslight=UCCB_PL_ON;
      } else if(g_key == '2') {
        l_sh1_poslight=UCCB_PL_BLINK;
      } else if(g_key == UCCB_KEY_ENTER) {
        g_sh1_poslight=l_sh1_poslight;
        l_mode=0;
        lcd.noBlink();
      }
    } else if(l_oi == 2) {
      if((g_key >= '0') && (g_key <= '9')) {
        if(l_sh1_maxmspeed < 0) {
          l_sh1_maxmspeed=(int)(g_key-'0');
        } else if(l_sh1_maxmspeed < 100) {
          l_sh1_maxmspeed=l_sh1_maxmspeed*10+(int)(g_key-'0');
        }
      } else if(g_key == UCCB_KEY_ENTER) {
        if((l_sh1_maxmspeed >= 50) && (l_sh1_maxmspeed <= 400)) {
          g_sh1_maxmspeed=l_sh1_maxmspeed;
          eval_setmaxspeed();
          l_mode=0;
          lcd.noBlink();
        }
      }
    } else if(l_oi == 3) {
      if((g_key >= '0') && (g_key <= '9')) {
        if(l_sh1_rotscale < 0) {
          l_sh1_rotscale=(int)(g_key-'0');
        } else if(l_sh1_rotscale < 10) {
          l_sh1_rotscale=l_sh1_rotscale*10+(int)(g_key-'0');
        }
      } else if(g_key == UCCB_KEY_ENTER) {
        if((l_sh1_rotscale >= 0) && (l_sh1_rotscale <= 20)) {
          g_sh1_rotscale=l_sh1_rotscale;
          g_sh1_rotcurve=0.05*g_sh1_rotscale;
          g_sh1_rotcurve=pow(10,g_sh1_rotcurve);
          l_mode=0;
          lcd.noBlink();
        }
      }
    } else if(l_oi == 4) {
      if(g_key == '1') {
        l_sh1_m2on=(l_sh1_m2on+1)%2;
      } else if(g_key == '3') {
        l_sh1_m1on=(l_sh1_m1on+1)%2;
      } else if(g_key == UCCB_KEY_ENTER) {
        g_sh1_m1on=l_sh1_m1on;
        g_sh1_m2on=l_sh1_m2on;
        l_mode=0;
        lcd.noBlink();
      }
    }
  }  
  
  if((force == 1) ||
//     (l_sh1_poslight != g_sh1_poslight) ||
     (l_key != g_key)) {

//Serial.println("xxxxxxxxxxx");

    diff=l_cli-l_oi;   
    l2p=1;    
    if((diff+1 >= 1) && (diff+1 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("PLight=");
      if(l_sh1_poslight == UCCB_PL_OFF) {
        lcd.print("Off  ");
      } else if(l_sh1_poslight == UCCB_PL_ON) {
        lcd.print("On   ");
      } else if(l_sh1_poslight == UCCB_PL_BLINK) {
        lcd.print("Blink");
      }
      lcd.print("(0/1/2)");
      l2p++;
    }
    
    if((diff+2 >= 1) && (diff+2 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("MSpeed=");
      if(l_sh1_maxmspeed >= 0) {
        printSpaces(l_sh1_maxmspeed,4);
      } else {
        lcd.print("    ");
      }
      lcd.print("<50,400>");
      l2p++;
    }
    
    if((diff+3 >= 1) && (diff+3 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("RScale=");
      if(l_sh1_rotscale >= 0) {
        printSpaces(l_sh1_rotscale,6);
      } else {
        lcd.print("      ");
      }
      lcd.print("<0,20>");
      l2p++;
    }
    
    if((diff+4 >= 1) && (diff+4 <= 3)) {
      lcd.setCursor(0,l2p);
      if(l_cli == l2p) {
        lcd.print("*");
      } else {
        lcd.print(" ");
      }
      lcd.print("mL=");
      if(l_sh1_m2on == 1) {
        lcd.print("On  ");
      } else {
        lcd.print("Off ");
      }
      lcd.print("mR=");
      if(l_sh1_m1on == 1) {
        lcd.print("On  ");
      } else {
        lcd.print("Off ");
      }
      lcd.print("(1/3)");
      l2p++;
    }
    
    if(l_oi == 1) {
      lcd.setCursor(0,l_cli);
    } else if(l_oi == 2) {
      if(l_sh1_maxmspeed >= 0) {
        lcd.setCursor(8+numPlaces(l_sh1_maxmspeed),l_cli);
      } else {
        lcd.setCursor(8,l_cli);
      }
    } else if(l_oi == 3) {
      if(l_sh1_rotscale >= 0) {
        lcd.setCursor(8+numPlaces(l_sh1_rotscale),l_cli);
      } else {
        lcd.setCursor(8,l_cli);
      }
    } else if(l_oi == 4) {
      lcd.setCursor(0,l_cli);
    }
    l_key=g_key;
  }
}

void scr_autocalib_txt(const char *head)
{
  lcd.setCursor(0,1);
  lcd.print(head);
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("Save(1)/Next(9)?    ");
  g_clb_phase++;
}

void scr_calib_txt(int *val, int *cp, int c_val, int e_val, const char *head)
{
  *val=c_val;
  lcd.setCursor(0,1);
  lcd.print(head);
  lcd.setCursor(0,2);
  *cp=lcd.print(e_val);
  (*cp)+=lcd.print(" ");
  lcd.print(*val);
  lcd.print(" ");
  lcd.print(c_val);
  lcd.print("  ");
  lcd.setCursor(0,3);
  lcd.print("Save(1)/Next(9)?    ");
  g_clb_phase++;
}

int scr_autocalib_clb(int x)
{
  uccb_clb_autocenter_TS();
  uccb_clb_autocenter_FS();
  return(0);  
}
      
int scr_calib_clb(int dir, int *val, int c_val, int cp, eepromsave_ft eepromsave_f)
{
  static int l_key=0;
  static unsigned long l_tmof1=0;
  static int l_stof1=0;
  
  if(dir > 0) {
    if(c_val > *val) *val=c_val;
  } else if(dir < 0) {
    if(c_val < *val) *val=c_val;
  } else {
    *val=c_val;
  }
  lcd.setCursor(cp,2);
  lcd.print(*val);
  lcd.print(" ");
  lcd.print(c_val);
  lcd.print("  ");
  if(l_key == '1') {
    if(g_millis > l_tmof1+300) {
      l_stof1=(l_stof1+1)%2;
      if(l_stof1 == 1) {
        lcd.setCursor(0,3);
        lcd.print("Save(1)/Next(9)?    ");
      } else {
        lcd.setCursor(0,3);
        lcd.print("Save(1)/Next(9)?1(#)");
      }
      l_tmof1=g_millis;
    }
  }
  if((g_key == '1') && (g_clb_phase%2 == 0)){
    l_key='1';
    return(0);
  }
  if((g_key == '#') && (l_key == '1') && (g_clb_phase%2 == 0)) {
    eepromsave_f(*val);
    eval_setup();
    g_clb_phase++;
    l_key=0;
    return(1);
  }
  if(g_key == '9') {
    l_key=0;
    if(g_clb_phase < UCCB_DSP_CLB_MAX) g_clb_phase++;
    return(0);
  }
  if(g_key == '8') {
    l_key=0;
    g_clb_phase=1;
    return(0);
  }
  if(g_key == '7') {
    l_key=0;
    if(g_clb_phase >= UCCB_DSP_CLB_MAX) {
      g_clb_phase=UCCB_DSP_CLB_FSMAXZ_TXT;
    } else if(g_clb_phase > 2) {
      if(g_clb_phase%2 == 0) {
        g_clb_phase-=3;
      } else {
        g_clb_phase-=2;
      }
    }
    return(0);
  }
  if((g_key != 0) && (l_key == '1')) {
    l_key=0;
    lcd.setCursor(0,3);
    lcd.print("Save(1)/Next(0)?    ");
  }
  return(0);
}

void scr_calib_eof(void)
{
  lcd.setCursor(0,1);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  lcd.print("End of calibration. ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
  if(g_key == '8') {
    g_clb_phase=1;
    lcd.setCursor(0,2);
    lcd.print("                    ");
  }
  if(g_key == '7') {
    if(g_clb_phase >= UCCB_DSP_CLB_MAX) {
      g_clb_phase=UCCB_DSP_CLB_FSMAXZ_TXT;
    } else if(g_clb_phase > 2) {
      if(g_clb_phase%2 == 0) {
        g_clb_phase-=3;
      } else {
        g_clb_phase-=2;
      }
    }
    lcd.setCursor(0,2);
    lcd.print("                    ");
  }
}

void dsp_scr_calibration(int force)
{
  static int l_val=0;
  static int l_cp=0;
  int ret,v;
  
  if(force == 1) {
  }
  switch(g_clb_phase) {
    case UCCB_DSP_CLB_AC_TXT:
      scr_autocalib_txt("Auto center:        ");
      break;
    case UCCB_DSP_CLB_AC_CLB:
      scr_calib_clb(0,&v,0,0,scr_autocalib_clb);
      break;
    case UCCB_DSP_CLB_TSCENX_TXT:
      scr_calib_txt(&l_val,&l_cp,g_tsX,uccb_clb_read_TSCENX(),"TS calibration cenX:");
      break;
    case UCCB_DSP_CLB_TSCENX_CLB:
      scr_calib_clb(0,&l_val,g_tsX,l_cp,uccb_clb_write_TSCENX);
      break;
    case UCCB_DSP_CLB_TSMINX_TXT:
      scr_calib_txt(&l_val,&l_cp,g_tsX,uccb_clb_read_TSMINX(),"TS calibration minX:");
      break;
    case UCCB_DSP_CLB_TSMINX_CLB:
      scr_calib_clb(-1,&l_val,g_tsX,l_cp,uccb_clb_write_TSMINX);
      break;
    case UCCB_DSP_CLB_TSMAXX_TXT:
      scr_calib_txt(&l_val,&l_cp,g_tsX,uccb_clb_read_TSMAXX(),"TS calibration maxX:");
      break;
    case UCCB_DSP_CLB_TSMAXX_CLB:
      scr_calib_clb(1,&l_val,g_tsX,l_cp,uccb_clb_write_TSMAXX);
      break;
    case UCCB_DSP_CLB_TSCENY_TXT:
      scr_calib_txt(&l_val,&l_cp,g_tsY,uccb_clb_read_TSCENY(),"TS calibration cenY:");
      break;
    case UCCB_DSP_CLB_TSCENY_CLB:
      scr_calib_clb(0,&l_val,g_tsY,l_cp,uccb_clb_write_TSCENY);
      break;
    case UCCB_DSP_CLB_TSMINY_TXT:
      scr_calib_txt(&l_val,&l_cp,g_tsY,uccb_clb_read_TSMINY(),"TS calibration minY:");
      break;
    case UCCB_DSP_CLB_TSMINY_CLB:
      scr_calib_clb(-1,&l_val,g_tsY,l_cp,uccb_clb_write_TSMINY);
      break;
    case UCCB_DSP_CLB_TSMAXY_TXT:
      scr_calib_txt(&l_val,&l_cp,g_tsY,uccb_clb_read_TSMAXY(),"TS calibration maxY:");
      break;
    case UCCB_DSP_CLB_TSMAXY_CLB:
      scr_calib_clb(1,&l_val,g_tsY,l_cp,uccb_clb_write_TSMAXY);
      break;
    case UCCB_DSP_CLB_FSCENX_TXT:
      scr_calib_txt(&l_val,&l_cp,g_fsX,uccb_clb_read_FSCENX(),"FS calibration cenX:");
      break;
    case UCCB_DSP_CLB_FSCENX_CLB:
      scr_calib_clb(0,&l_val,g_fsX,l_cp,uccb_clb_write_FSCENX);
      break;
    case UCCB_DSP_CLB_FSMINX_TXT:
      scr_calib_txt(&l_val,&l_cp,g_fsX,uccb_clb_read_FSMINX(),"FS calibration minX:");
      break;
    case UCCB_DSP_CLB_FSMINX_CLB:
      scr_calib_clb(-1,&l_val,g_fsX,l_cp,uccb_clb_write_FSMINX);
      break;
    case UCCB_DSP_CLB_FSMAXX_TXT:
      scr_calib_txt(&l_val,&l_cp,g_fsX,uccb_clb_read_FSMAXX(),"FS calibration maxX:");
      break;
    case UCCB_DSP_CLB_FSMAXX_CLB:
      scr_calib_clb(1,&l_val,g_fsX,l_cp,uccb_clb_write_FSMAXX);
      break;
    case UCCB_DSP_CLB_FSCENY_TXT:
      scr_calib_txt(&l_val,&l_cp,g_fsY,uccb_clb_read_FSCENY(),"FS calibration cenY:");
      break;
    case UCCB_DSP_CLB_FSCENY_CLB:
      scr_calib_clb(0,&l_val,g_fsY,l_cp,uccb_clb_write_FSCENY);
      break;
    case UCCB_DSP_CLB_FSMINY_TXT:
      scr_calib_txt(&l_val,&l_cp,g_fsY,uccb_clb_read_FSMINY(),"FS calibration minY:");
      break;
    case UCCB_DSP_CLB_FSMINY_CLB:
      scr_calib_clb(-1,&l_val,g_fsY,l_cp,uccb_clb_write_FSMINY);
      break;
    case UCCB_DSP_CLB_FSMAXY_TXT:
      scr_calib_txt(&l_val,&l_cp,g_fsY,uccb_clb_read_FSMAXY(),"FS calibration maxY:");
      break;
    case UCCB_DSP_CLB_FSMAXY_CLB:
      scr_calib_clb(1,&l_val,g_fsY,l_cp,uccb_clb_write_FSMAXY);
      break;
    case UCCB_DSP_CLB_FSCENZ_TXT:
      scr_calib_txt(&l_val,&l_cp,g_fsZ,uccb_clb_read_FSCENZ(),"FS calibration cenZ:");
      break;
    case UCCB_DSP_CLB_FSCENZ_CLB:
      scr_calib_clb(0,&l_val,g_fsZ,l_cp,uccb_clb_write_FSCENZ);
      break;
    case UCCB_DSP_CLB_FSMINZ_TXT:
      scr_calib_txt(&l_val,&l_cp,g_fsZ,uccb_clb_read_FSMINZ(),"FS calibration minZ:");
      break;
    case UCCB_DSP_CLB_FSMINZ_CLB:
      scr_calib_clb(-1,&l_val,g_fsZ,l_cp,uccb_clb_write_FSMINZ);
      break;
    case UCCB_DSP_CLB_FSMAXZ_TXT:
      scr_calib_txt(&l_val,&l_cp,g_fsZ,uccb_clb_read_FSMAXZ(),"FS calibration maxZ:");
      break;
    case UCCB_DSP_CLB_FSMAXZ_CLB:
      scr_calib_clb(1,&l_val,g_fsZ,l_cp,uccb_clb_write_FSMAXZ);
      break;
    default:
      scr_calib_eof();
      break;
  }
}

void dsp_scr_battshutdown(void)
{
  long r;

  if(tmr_do(&g_tmr_battV_shutdown) == 1) {
    if(g_tmr_battV_shutdown.cnt > 40) {
      r=(80-g_tmr_battV_shutdown.cnt)/4;
      if(r <= 0) {
        digitalWrite(UCCB_SHUTDOWN_PORT,HIGH);
      }
      if(g_tmr_battV_shutdown.cnt%2 == 0) {
        lcd.noBlink();
        lcd.setCursor(11,0);
        lcd.print('E');
        lcd.write((byte)0);
        lcd.write((byte)1);
        lcd.write((byte)2);
        lcd.print(r);
      } else {
        lcd.setCursor(11,0);
        lcd.print("     ");
      }
    }
  }
}
 
void dsp_print(void)
{
  static int l_sw10p=-1;
  int force=0;

  dsp_scr_battshutdown();
  if((l_sw10p != g_sw10p) && (g_sw10p > 0)) {
    lcd.clear();
    lcd.noBlink();
    l_sw10p=g_sw10p;
    force=1;
    dsp_scr_startup();
    g_clb_phase=1;
  }
  if(g_b6pBE == 11) {
    g_sh1_poslight=UCCB_PL_ON;
    if(g_sw10p == 8) {
      force=1;
    }
  } else if(g_b6pBE == 41) {
    g_sh1_poslight=UCCB_PL_OFF;
    if(g_sw10p == 8) {
      force=1;
    }
  }

  if(g_sw10p != 6) {
    finscan_move(0);
    finscan_cancel();
  }
  
  switch(g_sw10p) {
    case 1:
      dsp_scr_main(force);
      break;
    case 2:
      dsp_scr_pwr(force);
      break;
    case 3:
    dsp_scr_fingerstick(force);
      dsp_scr_thumbstick(force);
      break;
    case 4:
      dsp_scr_b6p(force);
      break;
    case 5:
      dsp_scr_calibration(force);
      break;
    case 6:
//      dsp_scr_fingerscan(force);
//      dsp_scr_adjps(force);
//      dsp_scr_tscr(force);
      dsp_scr_tools(force);
      break;
    case 8:
      dsp_scr_ship3(force);
      break;
    case 9:
      dsp_scr_ship2(force);
      break;
    case 10:
      dsp_scr_ship1(force);
      break;
    default:
      break;
  }
}

