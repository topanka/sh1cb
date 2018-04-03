/*
742 7.36V
745 7.39V
747 7.41V
751 7.46V
752 7.48V
754 7.50V
757 7.53V
760 7.57V
762 7.59V
811 8.22V
*/

int batt_setup(void)
{
  int rval=-1,i;
  uint8_t avn;

  ump_pinMode(UCCB_ADJPS_ON_UMPPORT,OUTPUT);

/*
  ump_digitalWrite(UCCB_ADJPS_ON_UMPPORT,UCCB_ADJPS_ON_OFF);
  g_adjps_on=0;
*/  
  

  ump_digitalWrite(UCCB_ADJPS_ON_UMPPORT,UCCB_ADJPS_ON_ON);
  g_adjps_on=1;


  ump_pinMode(UCCB_ADJPS_LEVEL_UMPPORT,OUTPUT);
  ump_digitalWrite(UCCB_ADJPS_LEVEL_UMPPORT,UCCB_ADJPS_LEVEL_HIGH);
  g_adjps_level=UCCB_ADJPS_LEVEL_HIGH;

  avn=7;
  smar_init(&g_smar_ADJV,SMAR_ADCLOC_ARDUINO,UCCB_ADJV_PORT,avn,5,3);
//fill the buffer  
  for(i=0;i < 2*avn;i++) {
    g_adjV=smar_analogRead(&g_smar_ADJV);
  }
  
  avn=7;
  smar_init(&g_smar_battV,SMAR_ADCLOC_ARDUINO,UCCB_BATTV_PORT,avn,5,3);
//fill the buffer  
  for(i=0;i < 2*avn;i++) {
    g_battV=smar_analogRead(&g_smar_battV);
  }
  tmr_init(&g_tmr_battV,100);
  
  avn=11;
  smar_init(&g_smar_battA,SMAR_ADCLOC_ARDUINO,UCCB_BATTA_PORT,avn,7,3);
//fill the buffer  
  for(i=0;i < 2*avn;i++) {
    g_battA=smar_analogRead(&g_smar_battA);
  }
  tmr_init(&g_tmr_battA,50);

  pinMode(UCCB_BATTSTATE_PORT,OUTPUT);
  digitalWrite(UCCB_BATTSTATE_PORT,LOW);

  avn=7;
  smar_init(&g_smar_3_3V,SMAR_ADCLOC_ARDUINO,UCCB_3_3V_PORT,avn,5,3);
//fill the buffer  
  for(i=0;i < 2*avn;i++) {
    g_3_3V=smar_analogRead(&g_smar_3_3V);
  }

  avn=7;
  smar_init(&g_smar_5_0V,SMAR_ADCLOC_ARDUINO,UCCB_5_0V_PORT,avn,5,3);
//fill the buffer  
  for(i=0;i < 2*avn;i++) {
    g_5_0V=smar_analogRead(&g_smar_5_0V);
  }
  
  pinMode(UCCB_SHUTDOWN_PORT,OUTPUT);
  tmr_init(&g_tmr_SD,250);
  
  rval=0;
  
end:

  return(rval);
}

int batt_read(void)
{
  unsigned char r; 
  unsigned long bltmo;
  static unsigned long l_bltmo=0;
  static unsigned char l_ledstate=0;

  if(tmr_do(&g_tmr_battV) == 1) {
    if(g_battV <= g_battV_shutdown) {
      if(g_tmr_battV_shutdown.init != 1) {
        tmr_init(&g_tmr_battV_shutdown,250);
      }
    } else {
      tmr_destroy(&g_tmr_battV_shutdown);
    }
    if((r=g_tmr_battV.cnt%16) == 0) {
      g_battV=smar_analogRead(&g_smar_battV);
    } else if(r == 4) {
      g_3_3V=smar_analogRead(&g_smar_3_3V);
    } else if(r == 8) {
      g_5_0V=smar_analogRead(&g_smar_5_0V);
    } else if(r == 12) {
      g_adjV=smar_analogRead(&g_smar_ADJV);
    }
    if(g_battV < g_battV_min) {
      if(l_bltmo == 0) {
        l_bltmo=g_millis;
      } else {
        if((g_millis-l_bltmo) > 5000) {
          if(g_battV < g_battV_crit) {
            if((l_ledstate == 0) && ((r == 0) || (r == 6))) {
              digitalWrite(UCCB_BATTSTATE_PORT,HIGH);
              l_ledstate=1;
            } else if((l_ledstate == 1) && ((r == 3) || (r == 9))) {
              digitalWrite(UCCB_BATTSTATE_PORT,LOW);
              l_ledstate=0;
            }
          } else {
            if((l_ledstate == 0) && (r == 0)) {
              digitalWrite(UCCB_BATTSTATE_PORT,HIGH);
              l_ledstate=1;
            } else if((l_ledstate == 1) && (r == 9)) {
              digitalWrite(UCCB_BATTSTATE_PORT,LOW);
              l_ledstate=0;
            }
          }
        }
      }
    } else {
      if(l_ledstate == 0) {
        if(l_bltmo == 0) {
          l_bltmo=g_millis;
        } else {
        if((g_millis-l_bltmo) > 5000) {
            digitalWrite(UCCB_BATTSTATE_PORT,HIGH);
            l_bltmo=0;
            l_ledstate=1;
          }
        }
      }
    }
  }
  if(tmr_do(&g_tmr_battA) == 1) {
    g_battA=smar_analogRead(&g_smar_battA);
  }

/*  
  temp00=analogRead(A9);
//  temp0=smar_analogRead(A9);
  temp0=analogRead(A9);
  ra_temp.addValue(temp0);
  temp=ra_temp.getAverage()+0.5;
  tt=ra_temp.getAverage()*0.48828125;
  Serial.print(temp00);
  Serial.print(" ");
  Serial.print(temp0);
  Serial.print(" ");
  Serial.print(temp);
  Serial.print(" ");
  temp=tt*10;
  Serial.print(temp/10);
  Serial.print(".");
  Serial.print(temp%10);
  Serial.println("C");
*/  

  return(0);
}

int batt_poweroff(void)
{
  static unsigned long l_tmr=0;
  
  if(g_sw10p == 1) {
    if((g_shutdown_state == 0) && (g_key == '0')) {
      if((g_millis-l_tmr) < 300) {
        g_shutdown_state=1;
      }
      l_tmr=g_millis;
    }
  }
  return(0);
}

int adjps_power_switch(void)
{
  int i;
  
  smar_reset(&g_smar_ADJV);
  if(g_adjps_on == 1) {
    ump_digitalWrite(UCCB_ADJPS_ON_UMPPORT,UCCB_ADJPS_ON_ON);
  } else {
    ump_digitalWrite(UCCB_ADJPS_ON_UMPPORT,UCCB_ADJPS_ON_OFF);
  }
  delay(250);
  for(i=0;i < 2*g_smar_ADJV.avn;i++) {
    g_adjV=smar_analogRead(&g_smar_ADJV);
  }

  return(0);
}

int adjps_level_switch(void)
{
  int i;
  
  smar_reset(&g_smar_ADJV);
  if(g_adjps_level == UCCB_ADJPS_LEVEL_HIGH) {
    ump_digitalWrite(UCCB_ADJPS_LEVEL_UMPPORT,UCCB_ADJPS_LEVEL_HIGH);
  } else {
    ump_digitalWrite(UCCB_ADJPS_LEVEL_UMPPORT,UCCB_ADJPS_LEVEL_LOW);
  }
  delay(250);
  for(i=0;i < 2*g_smar_ADJV.avn;i++) {
    g_adjV=smar_analogRead(&g_smar_ADJV);
  }

  return(0);
}

int tscr_power_switch(void)
{
  int i;
  
  if(g_tscr_on == UCCB_TSCR_ON_ON) {
    ump_digitalWrite(UCCB_TSCR_ON_UMPPORT,UCCB_TSCR_ON_ON);
  } else {
    ump_digitalWrite(UCCB_TSCR_ON_UMPPORT,UCCB_TSCR_ON_OFF);
  }
  
  return(0);

}

